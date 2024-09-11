from typing import List

import matplotlib.pyplot as plt
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import KSState, InitialState, PMState
from commonroad.visualization.draw_params import DynamicObstacleParams

# import CommonRoad-io modules
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Circle

### My Imports ###
from commonroad.scenario.obstacle import ObstacleRole
from commonroad.common.util import Interval
from commonroad.visualization.draw_params import OccupancyParams
################################


def create_trajectory_from_list_states(list_paths_primitives: List[List[PMState]]) -> Trajectory:
    # turns the solution (list of lists of states) into a CommonRoad Trajectory
    
    list_states = list()

    for path_primitive in list_paths_primitives:
        for state in path_primitive:
            kwarg = {
                  "time_step": state.time_step,
                  "position": state.position,
                  "velocity": state.velocity,
                  "velocity_y": state.velocity_y,
            }
            list_states.append(PMState(**kwarg))

    # remove duplicates. the primitive have 6 states, thus a duplicate appears every 6 states
#     list_states = [list_states[i] for i in range(len(list_states)) if i % 6 != 1]

    return Trajectory(
        initial_time_step=list_states[0].time_step, state_list=list_states
    )


def visualize_solution(
    scenario: Scenario, planning_problem_set: PlanningProblemSet, trajectory: Trajectory, waypoints, c
) -> None:
    from IPython import display

    num_time_steps = len(trajectory.state_list)
    # print(num_time_steps)
    
    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_initial_state = InitialState(
        # position=trajectory.state_list[0].position,
        position=planning_problem_set.initial_state.position if c == 0 else trajectory.state_list[0].position,
        orientation=trajectory.state_list[0].orientation,
        velocity=trajectory.state_list[0].velocity,
        time_step=trajectory.state_list[0].time_step,
        yaw_rate=0,
        slip_angle=0,
    )
    # print(dynamic_obstacle_initial_state)
    dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
    dynamic_obstacle_prediction = TrajectoryPrediction(
        trajectory, dynamic_obstacle_shape
    )
    
    # print(dynamic_obstacle_prediction)

    # generate the dynamic obstacle according to the specification
    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = ObstacleType.CAR
    ego_vehicle = DynamicObstacle(
        dynamic_obstacle_id,
        dynamic_obstacle_type,
        dynamic_obstacle_shape,
        dynamic_obstacle_initial_state,
        dynamic_obstacle_prediction,
    )

    # visualize scenario
    ego_params = DynamicObstacleParams()
    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"

    for i in range(0, num_time_steps):
        # print(ego_vehicle.state_at_time(i).position[0])   # longitudinal position
        # print(KSState.position) ###
        display.clear_output(wait=True)
        plt.figure(figsize=(25, 10))
        renderer = MPRenderer()
        renderer.draw_params.time_begin = trajectory.state_list[i].time_step
        scenario.draw(renderer)

        ego_params.time_begin = trajectory.state_list[i].time_step
        ego_params.trajectory.draw_trajectory = True
        ego_params.trajectory.facecolor = "#ff00ff"
        ego_params.trajectory.draw_continuous = True
        ego_params.trajectory.zorder = 60
        ego_params.trajectory.line_width = 1
        
        '''
        ### Drawing an obstacle ###
        obs_params = DynamicObstacleParams()
        obs_params.draw_icon = True
        obs_params.show_label = True
        obs_params.draw_direction = True
        # obs_params.vehicle_shape.occupancy.shape.facecolor = "green"

        x_interval = Interval(-20, 0.5)
        y_interval = Interval(-60, -20)

        obs = scenario.obstacles_by_position_intervals([x_interval, y_interval], \
                                                    (ObstacleRole.DYNAMIC, ObstacleRole.STATIC))

        occupancy = obs[7]

        occupancy.draw(renderer, draw_params=obs_params)
        ################################
        '''
        
        ### Drawing waypoints ###
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)
        ################################

        
        ego_vehicle.draw(renderer, draw_params=ego_params)
        planning_problem_set.draw(renderer)
        plt.gca().set_aspect("equal")
        renderer.render()
        plt.show()