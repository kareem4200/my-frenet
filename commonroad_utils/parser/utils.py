from typing import List
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, PMState
from commonroad.visualization.draw_params import DynamicObstacleParams, TrajectoryParams, MPDrawParams
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
    t_s,
    obstacles
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
    # print(excuted_trajectory.state_list[0].orientation)
    
    # create the ego vehicle prediction using the trajectory and the shape of the obstacle
    dynamic_obstacle_shape = Rectangle(width=1.0, length=3.3)
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
    # draw_params = MPDrawParams()
    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"
    # ego_params.draw_icon = True

    # Loop on the number of time steps in the excuted trajectory (currently 2)
    for i in range(0, num_time_steps):
        display.clear_output(wait=True)
        plt.figure(figsize=(25, 10))
        renderer = MPRenderer()
        renderer.focus_obstacle_id = dynamic_obstacle_id
        renderer.draw_params.time_begin = excuted_trajectory.state_list[i].time_step
        renderer.draw_params.dynamic_obstacle.draw_shape = True
        # renderer.draw_params.dynamic_obstacle.draw_icon = True
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
        
        # draw_params.dynamic_obstacle.show_label = False
        # draw_params.dynamic_obstacle.draw_icon = True
        # draw_params.dynamic_obstacle.draw_shape = True
        
        # Drawing waypoints
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)
            
        for o in obstacles:
            circle_1 = Circle(radius=0.5, center=np.array([*o[:2]]))
            circle_2 = Circle(radius=0.5, center=np.array([*o[2:]]))
            circle_1.draw(renderer)
            circle_2.draw(renderer)

        # Drawing and rendering
        drawn_trajectory.draw(renderer, draw_params=traj_params)
        ego_vehicle.draw(renderer, draw_params=ego_params)
        planning_problem_set.draw(renderer)
        plt.gca().set_aspect("equal")
        renderer.render()
        plt.show()
        

def create_video(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    excuted_trajectory: Trajectory,
    waypoints,
    obstacles):
    
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
    obs_params = DynamicObstacleParams()
    # traj_params = TrajectoryParams()
    # draw_params = MPDrawParams()
    ego_params.vehicle_shape.occupancy.shape.facecolor = "green"
    ego_params.draw_icon = True
    
    obs_params.vehicle_shape.occupancy.shape.facecolor = "blue"
    obs_params.draw_icon = True
    
    # display.clear_output(wait=True)
    plt.figure(figsize=(25, 10))
    ax = plt.gca()
    
    renderer = MPRenderer()
    renderer.focus_obstacle_id = dynamic_obstacle_id
    renderer.draw_params.time_begin = excuted_trajectory.state_list[0].time_step
    renderer.draw_params.dynamic_obstacle.draw_shape = True
    renderer.draw_params.dynamic_obstacle.draw_icon = True
    
    # scenario.draw(renderer)
    # planning_problem_set.draw(renderer)
    
    # circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
    # for c in circles:
    #     c.draw(renderer)
    
    # scenario.draw(renderer)
    # planning_problem_set.draw(renderer)
    
    def update(frame):
        # renderer.clear()
        # Drawing parameters of the excuted trajectory
        # ego_params.time_begin = excuted_trajectory.state_list[0].time_step
    
        ego_params.time_begin = frame
        ego_params.trajectory.draw_trajectory = True
        ego_params.trajectory.facecolor = "#ff00ff"
        ego_params.trajectory.draw_continuous = True
        ego_params.trajectory.zorder = 60
        ego_params.trajectory.line_width = 2
        
        obs_params.time_begin = frame
        # ego_params.trajectory.draw_trajectory = True
        # ego_params.trajectory.facecolor = "#ff00ff"
        # ego_params.trajectory.draw_continuous = True
        # ego_params.trajectory.zorder = 60
        # ego_params.trajectory.line_width = 2
        
        # Drawing parameters of the full trajectory
        # traj_params.draw_trajectory = True
        # traj_params.facecolor = "#6aa84f"
        # traj_params.draw_continuous = True
        # traj_params.zorder = 60
        # traj_params.line_width = 2
        
        # Drawing and rendering
        # drawn_trajectory.draw(renderer, draw_params=traj_params)
        
        scenario.lanelet_network.draw(renderer)
        planning_problem_set.draw(renderer)
        
        for s_obs in scenario.static_obstacles:
            s_obs.draw(renderer, draw_params=obs_params)
        
        for d_obs in scenario.dynamic_obstacles:
            d_obs.draw(renderer, draw_params=obs_params)
        
        circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
        for c in circles:
            c.draw(renderer)
            
        ego_vehicle.draw(renderer, draw_params=ego_params)
        plt.gca().set_aspect("equal")
        renderer.render()
        
    anim = FuncAnimation(ax.figure, 
                         update, 
                        #  frames=planning_problem_set.goal.state_list[0].time_step.end,
                        frames=num_time_steps+1,
                        # frames=5, 
                        #  init_func=init_frame, 
                         blit=False, 
                         interval=100)
    
    # anim.save('/home/kareem/my-frenet/commonroad_utils/scenarios_videos/crashed/' + scenario.scenario_id.__str__() + '.mp4', dpi=250, writer='ffmpeg')
    anim.save('/home/kareem/my-frenet/commonroad_utils/scenarios_videos/crashed/' + 'test' + '.mp4', dpi=250, writer='ffmpeg')
        
def visualize_scenario(
    scenario: Scenario, 
    planning_problem_set: PlanningProblemSet, 
    waypoints = 0, 
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

    plt.figure(figsize=(25, 10))
    renderer = MPRenderer()
    scenario.draw(renderer)
        
    # Drawing waypoints
    # circles = [Circle(radius = 0.5, center = np.array(wp)) for wp in waypoints]
    # for c in circles:
    #     c.draw(renderer)

    planning_problem_set.draw(renderer)
    plt.gca().set_aspect("equal")
    renderer.render()
    plt.show()