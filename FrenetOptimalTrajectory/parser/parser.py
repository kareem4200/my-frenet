from commonroad.scenario.obstacle import ObstacleRole
from commonroad.common.util import Interval
from commonroad.scenario.scenario import Scenario
from typing import List
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad.common.validity import is_integer_number
import numpy as np
from math import inf

class Parser():
      """
      Parses the required initial conditions for Frenet
      
      Args:
            scenario object.
            planning problem object.
            Optionally X and Y interval to search for obstacles in (-inf, inf).
      """
      def __init__(self, scenario, 
                   planning_problem, 
                   x_interval=Interval(-inf, inf), 
                   y_interval=Interval(-inf, inf)):
            
            self.scenario: Scenario = scenario
            # needs to be changed to list to support multiple planning problems
            self.planning_problem: PlanningProblem = planning_problem
            self.static_obstacles: List
            self.x_interval: Interval = x_interval
            self.y_interval: Interval = y_interval
      
      def filter_obstacles(self):
            """
            Filters scenario obstacles into static and dynamic obstacles lists
            
            Returns:
                  List of static obstacles objects.
                  List of dynamic obstacles objects.
            """
            static, dynamic = [], []
            obstacles = self.scenario.obstacles_by_position_intervals([self.x_interval, self.y_interval], \
                                                                  (ObstacleRole.DYNAMIC, ObstacleRole.STATIC))
            for obs in obstacles:
                  if obs.obstacle_role == ObstacleRole.STATIC:
                        static.append(obs)
                  elif obs.obstacle_role == ObstacleRole.DYNAMIC:
                        dynamic.append(obs)
                        
            return static, dynamic
      
      def parse_static_obstacles(self):
            """
            Gets the rear left and front right coordinates of the filtered static obstacles.
            
            Returns:
                  List of static obstacles coordinates.
            """
            s_obstacles, _ = Parser.filter_obstacles(self)
            s_obstacles_list = []
            for obs in s_obstacles:
                  occupancy = obs.occupancy_at_time(0)
                  if occupancy is not None:
                        vertices_np = occupancy._shape.vertices
                        vertices_list = vertices_np.tolist()
                        coordinates = vertices_list[1] + vertices_list[3]  # rear left & front right coordinates
                        s_obstacles_list.append(coordinates)
                  
            return s_obstacles_list
      
      def parse_dynamic_obstacles(self, time_step: int):
            """
            Gets the rear left and front right coordinates of the filtered dynamic obstacles.
            
            Args:
                  The desired time step.
            
            Returns:
                  List of static obstacles coordinates.
            """
            assert is_integer_number(time_step)
            _, d_obstacles = Parser.filter_obstacles(self)
            d_obstacles_list = []
            for obs in d_obstacles:
                  occupancy = obs.occupancy_at_time(time_step)
                  if occupancy is not None:
                        vertices_np = occupancy._shape.vertices
                        vertices_list = vertices_np.tolist()
                        coordinates = vertices_list[1] + vertices_list[3]  # rear left & front right coordinates
                        d_obstacles_list.append(coordinates)
                  
            return d_obstacles_list
      
      def parse_obstacles(self, time_step: int):
            """
            Calls parse_static_obstacles and parse_dynamic_obstacles.
            It parses the static obstacles only once at time step 0.
            
            Args:
                  The desired time step.
            
            Returns:
                  A combined list of static and dynamic obstacles.
            """
            if time_step == 0:
                  self.static_obstacles = self.parse_static_obstacles()
                  return self.static_obstacles + self.parse_dynamic_obstacles(time_step=time_step)
            else:
                  return self.static_obstacles + self.parse_dynamic_obstacles(time_step=time_step)
            
      def parse_initial_position(self, x_only: bool):
            """
            Gets the initial position of the Ego vehicle.
            
            Args:
                  A boolean value indicating whether to get X position only or X and Y position
            
            Returns:
                  A float of X position or an array of X and Y position.
            """
            if x_only:
                  return self.planning_problem.initial_state.position[0]
            else:
                  return self.planning_problem.initial_state.position
            
      def parse_velocity(self):
            vel_int = self.planning_problem.goal.state_list[0].velocity
            vel = (vel_int.start + vel_int.end) / 2
            return [vel, vel]
      
      def parse_waypoints(self, initial_state: np.array, goal_state: np.array):
            """
            Calculates a set of waypoints from the initial position to goal position.
            
            Args:
                  The initial state numpy array.
                  The goal state numpy array.
            
            Returns:
                  A list of X and Y waypoints.
            """
            route: Route = RoutePlanner(self.scenario, self.planning_problem).plan_routes().retrieve_best_route_by_orientation()
            instruction = route._compute_lane_change_instructions()
            list_portions = route._compute_lanelet_portion(instruction)
            reference_path = route._compute_reference_path(list_portions)
            
            distances = [np.linalg.norm(np.array(point) - np.array(initial_state)) for point in reference_path]
            start_index = distances.index(min(distances))
            
            distances = [np.linalg.norm(np.array(point) - np.array(goal_state)) for point in reference_path]
            goal_index = distances.index(min(distances))
            
            reference_path = reference_path[start_index:goal_index+1]
            
            return reference_path.tolist()

      