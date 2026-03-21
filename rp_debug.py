from commonroad_rp.utility.evaluation import create_full_solution_trajectory
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import os
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.scenario.state import InitialState
from commonroad.visualization.draw_params import DynamicObstacleParams, TrajectoryParams
from commonroad.visualization.mp_renderer import MPRenderer
matplotlib.use('TkAgg')

def visualize_solution(
    scenario, 
    planning_problem_set, 
    excuted_trajectory,
    t_s,
    ax,
) -> None:
    
    plt.ion()
    
#     num_time_steps = len(excuted_trajectory.state_list)
    num_time_steps = 1    
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

        # Drawing and rendering
        ego_vehicle.draw(renderer, draw_params=ego_params)
        planning_problem_set.draw(renderer)
        renderer.render()
        plt.pause(0.1)
        plt.gca().set_aspect("equal")
        plt.show()
        
        
# make sure to change the path of scenarios in reactive_planner_config.yaml
config_file = "/home/kareem/new_frenet/frenet_optimal_trajectory_planner/commonroad_utils/config/reactive_planner_config.yaml"
sc = '/home/kareem/new_frenet/frenet_optimal_trajectory_planner/commonroad_utils/Critical_Transformed/PRI_Barceloneta-4_5_T-1.xml'
# sc = "FRA_Miramas-9_1_T-1.xml"
# config = ReactivePlannerConfiguration.load(os.getcwd() + "/config/" + config_file, sc)
config = ReactivePlannerConfiguration.load(config_file, sc)
config.update()            

fig, ax = plt.subplots(figsize=(25, 10))

try:
      # run route planner
      route_planner = RoutePlanner(config.scenario, config.planning_problem)
      route = route_planner.plan_routes().retrieve_first_route()

      # get reference path
      reference_path = route.reference_path
      
      planner = ReactivePlanner(config=config)

      # set reference path for curvilinear coordinate system
      planner.set_reference_path(route.reference_path)
      
      i = 0
      while not planner.goal_reached():
            current_count = len(planner.record_state_list)
            # check if planning cycle or not
            plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
            

            # new planning cycle -> plan a new optimal trajectory
            if plan_new_trajectory:
                  # set velocity
                  planner.set_desired_velocity(current_speed=planner.x_0.velocity)

                  # call plan function
                  optimal = planner.plan()
                  
                  # print(optimal[0].state_list[1])
                  # print(planner.x_0.velocity)

                  # record planned state and input
                  planner.record_state_and_input(optimal[0].state_list[1])

                  # reset planner state for re-planning
                  planner.reset(initial_state_cart=planner.record_state_list[-1], 
                              initial_state_curv=(optimal[2][1], optimal[3][1]),
                              collision_checker=planner.collision_checker, 
                              coordinate_system=planner.coordinate_system)

            # simulate scenario one step forward with planned trajectory
            else:
                  # continue on optimal trajectory
                  temp = current_count % config.planning.replanning_frequency

                  # record state and input
                  planner.record_state_and_input(optimal[0].state_list[1 + temp])

                  # reset planner state for re-planning
                  planner.reset(initial_state_cart=planner.record_state_list[-1],
                              initial_state_curv=(optimal[2][1 + temp], optimal[3][1 + temp]),
                              collision_checker=planner.collision_checker, 
                              coordinate_system=planner.coordinate_system)
            
            trajectory = create_full_solution_trajectory(planner.config, [planner.record_state_list[-1]])
            visualize_solution(scenario=config.scenario, 
                               planning_problem_set=config.planning_problem,
                               excuted_trajectory=trajectory,
                               t_s=i,
                               ax=plt.gca()
                               )
            i += 1
      df = pd.DataFrame(planner._recorded_states, columns=['t', 'd', 'v', 'a', 'theta'])
      df.to_csv('cr_recorded_states.csv', index=False)
      
except Exception as e:
      print(e)
      print(f"Scenario {sc} failed!")