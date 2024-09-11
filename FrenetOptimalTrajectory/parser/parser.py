import os
import sys
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleRole, DynamicObstacle, StaticObstacle, Obstacle
from commonroad.common.util import Interval
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState, PMState
from commonroad.scenario.scenario import Scenario
from typing import List
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad.common.validity import is_integer_number
import numpy as np

class Parser():
      def __init__(self, scenario, planning_problem, x_interval, y_interval):
            self.scenario: Scenario = scenario
            # needs to be changed to list to support multiple planning problems
            self.planning_problem: PlanningProblem = planning_problem
            self.static_obstacles: List
            self.x_interval: Interval = x_interval
            self.y_interval: Interval = y_interval
      
      def filter_obstacles(self):
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
            if time_step == 0:
                  self.static_obstacles = self.parse_static_obstacles()
                  return self.static_obstacles + self.parse_dynamic_obstacles(time_step=time_step)
            else:
                  return self.static_obstacles + self.parse_dynamic_obstacles(time_step=time_step)
            
      def parse_initial_position(self, x_only: bool):
            if x_only:
                  return self.planning_problem.initial_state.position[0]
            else:
                  return self.planning_problem.initial_state.position
            
      def parse_velocity(self):
            vel_int = self.planning_problem.goal.state_list[0].velocity
            vel = (vel_int.start + vel_int.end) / 2
            return [vel, vel]
      
      def parse_waypoints(self, initial_state: np.array, goal_state: np.array):
            # print(self.planning_problem.initial_state.position)
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

      