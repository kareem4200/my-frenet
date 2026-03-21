import os
import sys
import time
import pickle
import pandas as pd
from math import inf
from typing import List
import numpy as np
import matplotlib.pyplot as plt
from Sobol import recordData
import commonroad
from commonroad_utils.parser.scenario import get_scenario
from commonroad_utils.parser.parser import Parser
from commonroad.common.util import Interval
from commonroad.scenario.state import ExtendedPMState, PMState
from commonroad_utils.parser.utils import *
# import sys
# sys.path.append('/home/wenguang/workplace/my-frenet/frenet_optimal_trajectory_planner/FrenetOptimalTrajectory')

os.environ["SHOW_SAMPLING_PATH"] = '1'

scenario_path = os.getcwd() + '/commonroad_utils/Critical_Transformed/'
scenario_name = 'ITA_Empoli-7_3_T-1.xml'

# # Check if a filename is provided as a command-line argument
# if len(sys.argv) < 2:
#     print("Usage: python main.py <scenario_filename>")
#     sys.exit(1)
# scenario_name = sys.argv[1]


scenario, planning_problem, pp_set = get_scenario(scenario_path, scenario_name)

# Get the initial and goal positions
start_pos = planning_problem.initial_state.position

# Scenario has no attribute position in goal
if not hasattr(planning_problem.goal.state_list[0], 'position'):
      print(f"Scenario {scenario_name} has no goal position.") 
      sys.exit(0)
# Scenario has attribute position as a lanelet in goal (get the center of the lanelet) 
elif isinstance(planning_problem.goal.state_list[0].position, commonroad.geometry.shape.ShapeGroup):
      goal_pos = planning_problem.goal.state_list[0].position.shapes[0].center
# Scenario has attribute position as a rectangle in goal (get the center of the rectangle)
elif isinstance(planning_problem.goal.state_list[0].position, commonroad.geometry.shape.Rectangle):
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

EPS = 1.5           # Epsilon of reached goal comparison
LEN_DRAW = 10     # The length of the drawn trajectory (Number of states)

SAVE = False
SAVE_CSV = False
LOAD = False
SAVE_PROFILES = False
CREATE_VIDEO = False
RECORD_DATA = True


# print(planning_problem.initial_state.acceleration)
conds = {
      's0': parser.parse_initial_position(x_only=True),
      'target_speed': 30.0,
      # 'target_speed': planner.x_0.velocity,  # Uncomment to parsing the target speed from the scenario
      'acc': planning_problem.initial_state.acceleration,
      'wp': parser.parse_waypoints(initial_state=start_pos, goal_state=goal_pos),
      'obs': parser.parse_obstacles(time_step=0),
      'pos': parser.parse_initial_position(x_only=False),
      'vel': [initial_vel_x, initial_vel_y], # Velocity in X and Y directions
      # 'vel': [-1.0, -12.0], # Velocity in X and Y directions
}

# print(initial_vel_x, initial_vel_y)

initial_conditions = {
      'ps': conds['s0'],
      'target_speed': conds['target_speed'],
      'pos': np.array(conds['pos']),
      'vel': np.array(conds['vel']),
      'acc': np.array(conds['acc']),
      'wp': np.array(conds['wp']),
      'obs': np.zeros((1, 8)),  # Uncomment to test with no obstacles
      # 'obs': np.array(conds['obs'])     # Comment to test with no obstacles
}

# print(np.array(conds['obs']) )
if SAVE:
      with open(f'commonroad_utils/frenet_ic/{scenario_name[:-4]}.pkl', 'wb') as f:
            pickle.dump(initial_conditions, f)
            
if LOAD:
      with open(f'commonroad_utils/frenet_ic/{scenario_name[:-4]}.pkl', 'rb') as f:
            initial_conditions = pickle.load(f)
            
ic_df = pd.DataFrame({
      'name': ['target_speed', 'vel_x', 'vel_y'],
      'value': [initial_conditions['target_speed'], initial_conditions['vel'][0], initial_conditions['vel'][1]]
})

hyperparameters = {
      "max_speed": 100.0,
      "max_accel": 15.0,
      "max_curvature": 10.0,
      "max_road_width_l": 1.75,
      "max_road_width_r": 1.75,
      "d_road_w": 0.25,
      "dt": 0.125,
      "maxt": 4.0,
      "mint": 1.0,
      "d_t_s": 2.0,
      "n_s_sample": 2.0,
      "obstacle_clearance": 0.0,
      "kd": 10,
      "kv": 0.125,
      "ka": 1,
      "kj": 0.125,
      "kt": 0.125,
      "ko": 0.125,
      "klat": 10.0,
      "klon": 1.0,
      "num_threads": 0
}

# if SAVE:
#       with open(f'commonroad_utils/frenet_hp/{scenario_name[:-4]}.pkl', 'wb') as f:
#             pickle.dump(hyperparameters, f)
            
if LOAD:
      with open(f'commonroad_utils/frenet_hp/{scenario_name[:-4]}.pkl', 'rb') as f:
            hyperparameters = pickle.load(f)
            
hp_df = pd.DataFrame(list(hyperparameters.items()), columns=['name', 'value'])
ic_df = pd.concat([ic_df, hp_df], ignore_index=True)

if SAVE_CSV:
      ic_df.to_csv(f'commonroad_utils/frenet_hp/{scenario_name[:-4]}.csv', index=False)

# print(hyperparameters)
# print(initial_conditions)

wp = initial_conditions["wp"]
# print(wp[-1])

# print(os.getpid())
acc_states = []
velocities_ = []
accelerations_ = []
sampled_paths_ = []
full_trajectories_ = []
fig, ax = plt.subplots(figsize=(25, 10))

show_sampling_path = os.environ.get("SHOW_SAMPLING_PATH", False)
from FrenetOptimalTrajectory.py_cpp_struct import FrenetReturnValues
from FrenetOptimalTrajectory import py_cpp_struct
from FrenetOptimalTrajectory import fot_wrapper

fot_wrapper.RECORD_DATA = RECORD_DATA
_timestep = 0
# Stopping = False
for i in range(200):
      fot_wrapper.step_num = _timestep
      # print(show_sampling_path)
      # Run Frenet planner
      if int(show_sampling_path):
            # print("Showing Sampling Path")
            result_x, result_y, speeds, accelerations, ix, iy, iyaw, d, s, speeds_x, \
                speeds_y, misc, costs, success, runtime_c, sample_x, sample_y = \
                fot_wrapper.run_fot(initial_conditions, hyperparameters)  
      else:
            result_x, result_y, speeds, accelerations, ix, iy, iyaw, d, s, speeds_x, \
                  speeds_y, misc, costs, success, runtime = \
                  fot_wrapper.run_fot(initial_conditions, hyperparameters)
      # if(success is False):
      #       initial_conditions['target_speed'] = 0
      #       Stopping = True
      #       continue

      # print(speeds[1])
      states_list: List[List[ExtendedPMState]] = [[ExtendedPMState(time_step=_timestep+j, position=np.array([result_x[j], result_y[j]]),\
                                                velocity=speeds[j], orientation=iyaw[j], acceleration=accelerations[j])] for j in range(len(result_x[:LEN_DRAW]))]
      # Create PMState for each sample path
      sampling_states = []
      if int(show_sampling_path):
            for path_x, path_y in zip(sample_x, sample_y):
                  path_states: List[List[ExtendedPMState]] = [[ExtendedPMState(time_step=_timestep+j, position=np.array([path_x[j], path_y[j]]), velocity=0, orientation=0, acceleration=0) \
                        for j in range(len(path_x[:LEN_DRAW]))]]
                  sampling_states.append(path_states)
            
      # Create trajectories from the sampling states
      sampling_paths = [create_trajectory_from_list_states(path_states) for path_states in sampling_states if path_states]
      sampled_paths_.append(sampling_paths)
      
      # Convert the list of PMstates to CommonRoad trajectory (full trajectory to draw it and a shorter one to excute it)
      try:
            full_trajectory = create_trajectory_from_list_states(states_list)
            excuted_trajectory = create_trajectory_from_list_states([states_list[1]])
      except Exception as e:
            print(e)
            print("Failed in create_trajectory_from_list_states()")
            break
      
      # Uncomment to print the planned states of the 2 time steps trajectory
      # for state in excuted_trajectory.state_list:
      #       print(state)
      
      # Uncomment to print the planned states of the full_trajectory
      # for state in full_trajectory.state_list:
      #       print(state)
      # print(speeds_x[1], speeds_y[1])
      
      # Visualize the scenario and trajectories
      # print(i)
      # print(parser.parse_obstacles(time_step=0))
      visualize_solution(scenario = scenario, 
                        planning_problem_set = planning_problem, 
                        drawn_trajectories = sampling_paths, 
                        excuted_trajectory = excuted_trajectory,
                        full_trajectory = full_trajectory,
                        waypoints = wp, 
                        t_s = _timestep,
                        ax=ax)
      
      # print("Euclidean Distance: ", np.linalg.norm(np.array([result_x[1], result_y[1]]) - goal_pos))
      acc_states.append(states_list[1][0])
      full_trajectories_.append(full_trajectory)
      _timestep += 1

      if success:
            # If planning suceeded, check if goal was reached. If true, break the planning loop
            if np.linalg.norm(np.array([result_x[1], result_y[1]]) - goal_pos) < EPS:
                  print("Goal Reached")
                  break
                  
            # Parse initial conditions again for the next time step
            initial_conditions['target_speed'] = np.sqrt(speeds_x[1]**2 + speeds_y[1]**2)
            # print(speeds[1])
            # time.sleep(3)
            initial_conditions['pos'] = np.array([result_x[1], result_y[1]])
            initial_conditions['ps'] = misc['s']
            initial_conditions['vel'] = np.array([speeds_x[1], speeds_y[1]])
            initial_conditions['acc'] = np.array(accelerations[1])
            initial_conditions['obs'] = np.zeros((1, 8))      # Uncomment to test with no obstacles
            # initial_conditions['obs'] = np.array(parser.parse_obstacles(time_step=_timestep+1))         # Comment to test with no obstacles
            velocities_.append(speeds[1])
            accelerations_.append(accelerations[1])
      else:
            # continue
            print("Failed unexpectedly")
            break
recordData.saveInitialConditions()
recordData.saveCalculatedData()
recordData.saveHyperparameter()
      
# print(acc_states)
excuted_trajectory = create_trajectory_from_list_states([acc_states])

if CREATE_VIDEO:
      create_video(
            scenario=scenario,
            planning_problem_set=planning_problem,
            excuted_trajectory=excuted_trajectory,
            full_trajectory=full_trajectories_,
            sampled_trajectories=sampled_paths_,
            waypoints=wp
      )

if SAVE_PROFILES:
      plot_profile(velocities_, 'Velocity', scenario_name, SAVE_PROFILES)
      plot_profile(accelerations_, 'Acceleration', scenario_name, SAVE_PROFILES)
