import os
from math import inf
from typing import List
import numpy as np
import matplotlib.pyplot as plt
from commonroad_utils.parser.scenario import get_scenario
from commonroad_utils.parser.parser import Parser
from commonroad.common.util import Interval
from commonroad.scenario.state import PMState
from commonroad_utils.parser.utils import create_trajectory_from_list_states, visualize_solution, visualize_scenario, create_video
# import sys
# sys.path.append('/home/wenguang/workplace/my-frenet/frenet_optimal_trajectory_planner/FrenetOptimalTrajectory')

os.environ["SHOW_SAMPLING_PATH"] = "1"

scenario_path = os.getcwd() + '/commonroad_utils/Critical_Transformed/'
scenario_name = 'ITA_Empoli-7_3_T-1.xml'
# scenario_name = 'USA_US101-28_1_T-1.xml'

scenario, planning_problem, pp_set = get_scenario(scenario_path, scenario_name)

# Get the initial and goal positions
start_pos = planning_problem.initial_state.position
goal_pos = planning_problem.goal.state_list[0].position.center
# print(start_pos, goal_pos)
# Calculate X and Y components on the initial velocity
inital_vel = planning_problem.initial_state.velocity
initial_orientation = planning_problem.initial_state.orientation
initial_vel_x = inital_vel * np.cos(initial_orientation)
initial_vel_y = inital_vel * np.sin(initial_orientation)
# print(initial_vel_x, initial_vel_y)
parser = Parser(scenario=scenario, planning_problem=planning_problem,\
                  x_interval=Interval(-inf, inf), y_interval=Interval(-inf, inf))

EPS = 0.5           # Epsilon of reached goal comparison
LEN_DRAW = 10     # The length of the drawn trajectory (Number of states)

conds = {
      's0': parser.parse_initial_position(x_only=True),
      'target_speed': 15.0,
      # 'target_speed': planner.x_0.velocity,  # Uncomment to parsing the target speed from the scenario
      'wp': parser.parse_waypoints(initial_state=start_pos, goal_state=goal_pos),
      'obs': parser.parse_obstacles(time_step=0),
      'pos': parser.parse_initial_position(x_only=False),
      # 'vel': [initial_vel_x, initial_vel_y], # Velocity in X and Y directions
      'vel': [1, 1], # Velocity in X and Y directions
}

initial_conditions = {
      'ps': conds['s0'],
      'target_speed': conds['target_speed'],
      'pos': np.array(conds['pos']),
      'vel': np.array(conds['vel']),
      'wp': np.array(conds['wp']),
      # 'obs': np.array([]),  # Uncomment to test with no obstacles
      'obs': np.array(conds['obs'])     # Comment to test with no obstacles
}

hyperparameters = {
      "max_speed": 100.0,
      "max_accel": 15.0,
      "max_curvature": 100.0,
      "max_road_width_l": 2,
      "max_road_width_r": 2,
      "d_road_w": 0.2,
      "dt": 0.2,
      "maxt": 2.0,
      "mint": 1.0,
      "d_t_s": 0.5,
      "n_s_sample": 2.0,
      "obstacle_clearance": -0.5,
      "kd": 0.1,
      "kv": 0.1,
      "ka": 0.1,
      "kj": 0.1,
      "kt": 0.1,
      "ko": 0.1,
      "klat": 1.0,
      "klon": 1.0,
      "num_threads": 0
}

wp = initial_conditions["wp"]
# print(wp)

print(os.getpid())
acc_states = []
fig, ax = plt.subplots(figsize=(25, 10))

show_sampling_path = os.environ.get("SHOW_SAMPLING_PATH", False)
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.py_cpp_struct import FrenetReturnValues
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import py_cpp_struct
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import fot_wrapper

for i in range(200):
      # Run Frenet planner
      if show_sampling_path:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c, sample_x, sample_y = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)  
      else:
            result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)
                
      states_list: List[List[PMState]] = [[PMState(time_step=i+j, position=np.array([result_x[j], result_y[j]]),\
                                                velocity=speeds_x[j], velocity_y=speeds_y[j])] for j in range(len(result_x[:LEN_DRAW]))]
      # Create PMState for each sample path
      sampling_states = []
      for path_x, path_y in zip(sample_x, sample_y):
            path_states: List[List[PMState]] = [[PMState(time_step=i+j, position=np.array([path_x[j], path_y[j]]), velocity=0, velocity_y=0) for j in range(len(path_x[:LEN_DRAW]))]]
            sampling_states.append(path_states)
      
      # Create trajectories from the sampling states
      sampling_paths = [create_trajectory_from_list_states(path_states) for path_states in sampling_states if path_states]
      
      # Convert the list of PMstates to CommonRoad trajectory (full trajectory to draw it and a shorter one to excute it)
      try:
            full_trajectory = create_trajectory_from_list_states(states_list)
            excuted_trajectory = create_trajectory_from_list_states([states_list[1]])
      except Exception as e:
            print(e)
            break
      
      # Uncomment to print the planned states of the 2 time steps trajectory
      # for state in excuted_trajectory.state_list:
      #       print(state)
      
      # Uncomment to print the planned states of the full_trajectory
      # for state in full_trajectory.state_list:
      #       print(state)
      
      # Visualize the scenario and trajectories
      visualize_solution(scenario = scenario, 
                        planning_problem_set = planning_problem, 
                        drawn_trajectories = sampling_paths, 
                        excuted_trajectory = excuted_trajectory,
                        full_trajectory = full_trajectory,
                        waypoints = wp, 
                        t_s = i,
                        fig=fig,
                        ax=ax)
      
      # print("Euclidean Distance: ", np.linalg.norm(np.array([result_x[1], result_y[1]]) - goal_pos))

      if success:
            # If planning suceeded, check if goal was reached. If true, break the planning loop
            if np.linalg.norm(np.array([result_x[1], result_y[1]]) - goal_pos) < EPS:
                  print("Goal Reached")
                  break
                  
            # Parse initial conditions again for the next time step
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['ps'] = misc['s']
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['obs'] = np.array([])      # Uncomment to test with no obstacles
            # initial_conditions['obs'] = np.array(parser.parse_obstacles(time_step=i+1))         # Comment to test with no obstacles
            acc_states.append(states_list[1][0])
      else:
            # continue
            print("Failed unexpectedly")
            break
      
# print(acc_states)
# excuted_trajectory = create_trajectory_from_list_states([acc_states])
# create_video(
#       scenario=scenario,
#       planning_problem_set=planning_problem,
#       excuted_trajectory=excuted_trajectory,
#       waypoints=wp
# )
