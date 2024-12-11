from typing import List
import matplotlib.pyplot as plt
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, PMState
from commonroad.visualization.draw_params import DynamicObstacleParams, TrajectoryParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Circle
from IPython import display

def create_trajectory_from_list_states(list_paths_primitives: List[List[PMState]]) -> Trajectory:
    # turns the solution (list of lists of states) into a CommonRoad Trajectory
    """
    Turns the solution (list of lists of states) into a CommonRoad Trajectory.
    
    Args:
        List of lists of states generated from Frenet.
        
    Returns:
        A CommonRoad Trajectory object.
    """
    
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

    return Trajectory(
        initial_time_step=list_states[0].time_step, state_list=list_states
    )

def visualize_solution(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    drawn_trajectory: Trajectory, 
    excuted_trajectory: Trajectory,
    waypoints, 
    t_s
) -> None:
    """
    Plots the scenario, planning problem, waypoints, Ego vehicle, excuted, and full trajectory.
    
    Args:
        Scenario object.
        Planning problem object.
        The drawn trajectory.
        The excuted trajectory.
        List of waypoints.
        The current time step.
    """

    num_time_steps = len(excuted_trajectory.state_list)
    
    # defines the initial state of the ego vehicle (changes each planning step)
    dynamic_obstacle_initial_state = InitialState(
        # position=trajectory.state_list[0].position,
        position=planning_problem_set.initial_state.position if t_s == 0 else excuted_trajectory.state_list[0].position,
        orientation=excuted_trajectory.state_list[0].orientation,
        velocity=excuted_trajectory.state_list[0].velocity,
        time_step=excuted_trajectory.state_list[0].time_step,
        yaw_rate=0,
        slip_angle=0,
    )
    print(excuted_trajectory.state_list[0].orientation)
    
    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_shape = Rectangle(width=1.8, length=4.3)
    dynamic_obstacle_prediction = TrajectoryPrediction(
        excuted_trajectory, dynamic_obstacle_shape
    )
    
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

    # Initialize the vehicle and trajectory drawing parameters
    ego_params = DynamicObstacleParams()
    traj_params = TrajectoryParams()
    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"

    # Loop on the number of time steps in the excuted trajectory (currently 2)
    for i in range(0, num_time_steps):
        display.clear_output(wait=True)
        plt.figure(figsize=(25, 10))
        renderer = MPRenderer()
        renderer.draw_params.time_begin = excuted_trajectory.state_list[i].time_step
        scenario.draw(renderer)

        # Drawing parameters of the excuted trajectory
        ego_params.time_begin = excuted_trajectory.state_list[i].time_step
        ego_params.trajectory.draw_trajectory = True
        ego_params.trajectory.facecolor = "#ff00ff"
        ego_params.trajectory.draw_continuous = True
        ego_params.trajectory.zorder = 60
        ego_params.trajectory.line_width = 2
        
        # Drawing parameters of the full trajectory
        traj_params.draw_trajectory = True
        traj_params.facecolor = "#6aa84f"
        traj_params.draw_continuous = True
        traj_params.zorder = 60
        traj_params.line_width = 2
        
        # Drawing waypoints
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)

        # Drawing and rendering
        drawn_trajectory.draw(renderer, draw_params=traj_params)
        ego_vehicle.draw(renderer, draw_params=ego_params)
        planning_problem_set.draw(renderer)
        plt.gca().set_aspect("equal")
        renderer.render()
        plt.show()