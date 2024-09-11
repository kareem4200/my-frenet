import os
import sys
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.obstacle import ObstacleRole, DynamicObstacle, ObstacleType
from commonroad.common.util import Interval
from commonroad.visualization.draw_params import DynamicObstacleParams
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState, PMState
from commonroad.scenario.scenario import Scenario

def get_scenario(path: str, file_name: str):
      # id_scenario = 'USA_Lanker-1_2_T-1'
      # read in scenario and planning problem set
      # scenario, planning_problem_set = CommonRoadFileReader(os.path.join(path, file_name)).open()
      scenario, planning_problem_set = CommonRoadFileReader(file_name).open()
      # retrieve the first planning problem in the problem set
      planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
      
      return scenario, planning_problem, planning_problem_set

def agent(scenario: Scenario, initial_pos, initial_vel_x, initial_vel_y):
      dynamic_obstacle_initial_state = PMState(
        position=initial_pos,
        velocity=initial_vel_x,
        velocity_y=initial_vel_y)
      
      dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
      
#       dynamic_obstacle_prediction = TrajectoryPrediction(
#         trajectory, dynamic_obstacle_shape
#     )
    
    # generate the dynamic obstacle according to the specification
      dynamic_obstacle_id = scenario.generate_object_id()
      dynamic_obstacle_type = ObstacleType.CAR
      ego_vehicle = DynamicObstacle(
            dynamic_obstacle_id,
            dynamic_obstacle_type,
            dynamic_obstacle_shape,
            dynamic_obstacle_initial_state,
            # dynamic_obstacle_prediction,
      )

      # visualize scenario
      ego_params = DynamicObstacleParams()
      ego_params.vehicle_shape.occupancy.shape.facecolor = "green"






