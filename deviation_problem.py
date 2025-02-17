import os
from math import inf
from typing import List
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import fot_wrapper
import numpy as np
from commonroad_utils.parser.scenario import get_scenario
from commonroad_utils.parser.parser import Parser
from commonroad.common.util import Interval
from commonroad.scenario.state import PMState
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.common.file_reader import CommonRoadFileReader
# from commonroad_rp.utility.config import ReactivePlannerConfiguration
# from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_utils.parser.utils import create_trajectory_from_list_states, visualize_solution, visualize_scenario, create_video


scenario_path = os.getcwd() + '/commonroad_utils/Critical_Transformed/'
scenario_name = 'ITA_Empoli-7_3_T-1.xml'
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
      'target_speed': 20.0,
      # 'target_speed': planner.x_0.velocity,  # Uncomment to parsing the target speed from the scenario
      'wp': parser.parse_waypoints(initial_state=start_pos, goal_state=goal_pos),
      'obs': parser.parse_obstacles(time_step=0),
      'pos': parser.parse_initial_position(x_only=False),
      'vel': [initial_vel_x, initial_vel_y], # Velocity in X and Y directions
      # 'vel': [-10, -0.3], # Velocity in X and Y directions
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
      "max_speed": conds['target_speed'],
      "max_accel": 10.0,
      "max_curvature": 5.0,
      "max_road_width_l": 1.75,
      "max_road_width_r": 1.0,
      "d_road_w": 0.2,
      "dt": 0.1,
      "maxt": 2.0,
      "mint": 1.0,
      "d_t_s": 0.5,
      "n_s_sample": 20.0,
      "obstacle_clearance": -0.5,
      "kd": 100.0,
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

for i in range(200):
      # Run Frenet planner
      result_x, result_y, speeds, ix, iy, iyaw, d, s, speeds_x, \
                  speeds_y, misc, costs, success, runtime = fot_wrapper.run_fot(initial_conditions, hyperparameters)
            
      states_list: List[List[PMState]] = [[PMState(time_step=i+j, position=np.array([result_x[j], result_y[j]]),\
                                                velocity=speeds_x[j], velocity_y=speeds_y[j])] for j in range(len(result_x[:LEN_DRAW]))]
      
      # print(success)
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
                        drawn_trajectory = full_trajectory, 
                        excuted_trajectory = excuted_trajectory,
                        waypoints = wp, 
                        t_s = i,
                        obstacles=initial_conditions["obs"])
      
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
            # initial_conditions['obs'] = np.array([])      # Uncomment to test with no obstacles
            initial_conditions['obs'] = np.array(parser.parse_obstacles(time_step=i+1))         # Comment to test with no obstacles
            acc_states.append(states_list[1][0])
      else:
            # continue
            print("Failed unexpectedly")
            break
      
# renderer.create_video([scenario, planning_problem], str(scenario.scenario_id) + ".gif", fig_size = [15, 8], draw_params=renderer.draw_params)
# print(acc_states)
# excuted_trajectory = create_trajectory_from_list_states([acc_states])
# create_video(
#       scenario=scenario,
#       planning_problem_set=planning_problem,
#       excuted_trajectory=excuted_trajectory,
#       waypoints=wp
# )
