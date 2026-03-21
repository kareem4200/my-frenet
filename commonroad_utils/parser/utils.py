from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import os
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, PMState, ExtendedPMState
from commonroad.visualization.draw_params import DynamicObstacleParams, TrajectoryParams, MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Circle
from IPython import display

def create_trajectory_from_list_states(list_paths_primitives: List[List[ExtendedPMState]]) -> Trajectory:
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
            # kwarg = {
            #       "time_step": state.time_step,
            #       "position": state.position,
            #       "velocity": state.velocity,
            #       "velocity_y": state.velocity_y,
            # }
            kwarg = {
                  "time_step": state.time_step,
                  "position": state.position,
                  "velocity": state.velocity,
                  "orientation": state.orientation,
                  "acceleration": state.acceleration,
                #   "velocity_y": state.velocity_y,
            }
            list_states.append(ExtendedPMState(**kwarg))

    return Trajectory(
        initial_time_step=list_states[0].time_step, state_list=list_states
    )

def visualize_solution(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    excuted_trajectory: Trajectory,
    full_trajectory: Trajectory,
    waypoints, 
    t_s,
    ax,
    drawn_trajectories: List[Trajectory] = None, 
    obstacles = None
) -> None:
    """
    Plots the scenario, planning problem, waypoints, Ego vehicle, excuted, and full trajectory.
    
    Args:
        Scenario object.
        Planning problem object.
        List of drawn trajectories.
        The excuted trajectory.
        List of waypoints.
        The current time step.
    """
    
    plt.ion()
    
    num_time_steps = len(excuted_trajectory.state_list)
    
    # defines the initial state of the ego vehicle (changes each planning step)
    dynamic_obstacle_initial_state = InitialState(
        # position=trajectory.state_list[0].position,
        position=planning_problem_set.initial_state.position if t_s == 0 else excuted_trajectory.state_list[0].position,
        orientation=excuted_trajectory.state_list[0].orientation,
        velocity=excuted_trajectory.state_list[0].velocity,
        time_step=excuted_trajectory.state_list[0].time_step,
        acceleration=excuted_trajectory.state_list[0].acceleration,
        yaw_rate=0,
        slip_angle=0,
    )
    
    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_shape = Rectangle(width=1.86, length=4.93)
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
    sampled_traj_params = TrajectoryParams()
    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"
    ego_params.draw_icon = True
    
    # Loop on the number of time steps in the excuted trajectory
    for i in range(0, num_time_steps):
        ax.cla()
        
        renderer = MPRenderer(ax=ax)
        renderer.ax = ax
        renderer.focus_obstacle_id = dynamic_obstacle_id
        renderer.draw_params.time_begin = excuted_trajectory.state_list[i].time_step
        renderer.draw_params.dynamic_obstacle.draw_shape = True
        renderer.draw_params.dynamic_obstacle.draw_icon = True
        scenario.draw(renderer)

        # Drawing parameters of the excuted trajectory
        ego_params.time_begin = excuted_trajectory.state_list[i].time_step
        ego_params.trajectory.draw_trajectory = True
        ego_params.trajectory.facecolor = "#ff00ff"
        ego_params.trajectory.draw_continuous = True
        ego_params.trajectory.zorder = 60
        ego_params.trajectory.line_width = 2
        
        # Drawing parameters of the full trajectories
        traj_params.draw_trajectory = True
        traj_params.facecolor = "#6aa84f"
        traj_params.draw_continuous = True
        traj_params.zorder = 60
        traj_params.line_width = 2
        
        sampled_traj_params.draw_trajectory = True
        sampled_traj_params.facecolor = "#808080"
        sampled_traj_params.draw_continuous = True
        sampled_traj_params.zorder = 60
        sampled_traj_params.line_width = 2
        
        # Drawing waypoints
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)
        
        if obstacles is not None: 
            for o in obstacles:
                circle_1 = Circle(radius=0.5, center=np.array([*o[:2]]))
                circle_2 = Circle(radius=0.5, center=np.array([*o[2:4]]))
                circle_3 = Circle(radius=0.5, center=np.array([*o[4:6]]))
                circle_4 = Circle(radius=0.5, center=np.array([*o[6:8]]))
                
                circle_1.draw(renderer)
                circle_2.draw(renderer)
                circle_3.draw(renderer)
                circle_4.draw(renderer)

        # Drawing and rendering
        if drawn_trajectories:
            for drawn_trajectory in drawn_trajectories:
                drawn_trajectory.draw(renderer, draw_params=sampled_traj_params)
        full_trajectory.draw(renderer, draw_params=traj_params)
        ego_vehicle.draw(renderer, draw_params=ego_params)
        planning_problem_set.draw(renderer)
        renderer.render()
        plt.pause(0.1)
        plt.gca().set_aspect("equal")
        plt.show()
        
        
def create_video(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    excuted_trajectory: Trajectory,
    full_trajectory: List[Trajectory],
    sampled_trajectories: List[List[Trajectory]],
    waypoints):
    
    '''
    Creates a video of the scenario, planning problem, waypoints, and the excuted trajectory.
    
    args:
        scenario: Scenario object.
        planning_problem_set: PlanningProblemSet object.
        excuted_trajectory: Trajectory object.
        full_trajectory: Trajectory object.
        sampled_trajectories: List of lists of Trajectory objects.
        waypoints: List of waypoints.
    '''
    
    num_time_steps = len(excuted_trajectory.state_list)
    
    # defines the initial state of the ego vehicle (changes each planning step)
    dynamic_obstacle_initial_state = InitialState(
        # position=trajectory.state_list[0].position,
        position=planning_problem_set.initial_state.position,
        orientation=excuted_trajectory.state_list[0].orientation,
        velocity=excuted_trajectory.state_list[0].velocity,
        time_step=excuted_trajectory.state_list[0].time_step,
        yaw_rate=0,
        slip_angle=0,
    )
    # print(excuted_trajectory.state_list[0].orientation)
    
    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_shape = Rectangle(width=1.86, length=4.93)
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
    obs_params = DynamicObstacleParams()
    traj_params = TrajectoryParams()
    sampled_traj_params = TrajectoryParams()

    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"
    ego_params.draw_icon = True
    
    obs_params.vehicle_shape.occupancy.shape.facecolor = "blue"
    obs_params.draw_icon = True
    
    traj_params.draw_trajectory = False
    traj_params.facecolor = "#6aa84f"
    traj_params.draw_continuous = True
    traj_params.zorder = 60
    traj_params.line_width = 2
    
    sampled_traj_params.draw_trajectory = True
    sampled_traj_params.facecolor = "#808080"
    sampled_traj_params.draw_continuous = True
    sampled_traj_params.zorder = 60
    sampled_traj_params.line_width = 2
    
    # display.clear_output(wait=True)
    plt.figure(figsize=(25, 10))
    ax = plt.gca()
    
    renderer = MPRenderer()
    renderer.focus_obstacle_id = dynamic_obstacle_id
    renderer.draw_params.time_begin = excuted_trajectory.state_list[0].time_step
    renderer.draw_params.dynamic_obstacle.draw_shape = True
    renderer.draw_params.dynamic_obstacle.draw_icon = True
    
    def update(frame):
        ego_params.time_begin = frame
        ego_params.trajectory.draw_trajectory = False
        ego_params.trajectory.facecolor = "#ff00ff"
        ego_params.trajectory.draw_continuous = False
        ego_params.trajectory.zorder = 60
        ego_params.trajectory.line_width = 4
        
        obs_params.time_begin = frame
        
        scenario.lanelet_network.draw(renderer)
        planning_problem_set.draw(renderer)
        
        for s_obs in scenario.static_obstacles:
            s_obs.draw(renderer, draw_params=obs_params)
        
        for d_obs in scenario.dynamic_obstacles:
            d_obs.draw(renderer, draw_params=obs_params)
        
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)
                    
        if sampled_trajectories:
            for sample in sampled_trajectories[frame]:
                sample.draw(renderer, draw_params=sampled_traj_params)
                
        full_trajectory[frame].draw(renderer, draw_params=traj_params)
            
        ego_vehicle.draw(renderer, draw_params=ego_params)
        plt.gca().set_aspect("equal")
        renderer.render()
        
    anim = FuncAnimation(ax.figure, 
                         update, 
                         frames=num_time_steps,
                         interval=200)
    
    anim.save('/home/kareem/frenet_optimal_trajectory_planner/commonroad_utils/scenarios_videos/' + scenario.scenario_id.__str__() + '.mp4', dpi=250, writer='ffmpeg')
    # anim.save('/home/kareem/frenet_optimal_trajectory_planner/commonroad_utils/scenarios_videos/crashed/' + 'test' + '.mp4', dpi=250, writer='ffmpeg')
        
def visualize_scenario(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    waypoints = None, 
) -> None:
    """
    Plots the scenario, planning problem, and waypoints.
    
    Args:
        Scenario object.
        Planning problem object.
        Waypoints (Optional).
    """

    plt.figure(figsize=(25, 10))
    renderer = MPRenderer()
    scenario.draw(renderer)
        
    # Drawing waypoints
    if waypoints is not None:
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)

    planning_problem_set.draw(renderer)
    plt.gca().set_aspect("equal")
    renderer.render()
    plt.show()
    
def plot_profile(
    attribute,
    name,
    scenario_name,
    save
):
    """
    Plots the profile of the attribute against time steps.
    
    Args:
        The attribute to plot.
        The name of the attribute
        Scenario name.
        Save the plot in the profiles folder.
    """
    
    plt.figure(figsize=(10, 5))
    plt.plot(range(len(attribute)), attribute)
    plt.xlabel('Time Steps')
    plt.ylabel(f'{name}')
    plt.title(f'{name} Profile')
    
    plt.grid(True)
    
    path = os.getcwd() + '/commonroad_utils/profiles/' + scenario_name
    
    if save:
        if not os.path.exists(path):
            os.makedirs(path)
        plt.savefig(path + f'/{name}_profile.png')