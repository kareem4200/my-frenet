import os
# import glob
import numpy as np
import pandas as pd
import commonroad
from commonroad.scenario.state import InitialState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from cvae.model.utils import save_scenario_imgs

from commonroad_rp.utility.evaluation import create_full_solution_trajectory
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import commonroad_rp
print(commonroad_rp.__file__)
import matplotlib
matplotlib.use('Agg')  # Non-GUI backend (renders to image files)
from tqdm import tqdm

dir = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/scenarios"
config_file = "/home/kareem/frenet_optimal_trajectory_planner/CVAE/config/reactive_planner_config.yaml"

# planned = np.array([])
# planned = np.load("CVAE/data/planned_scenarios.npy")
# print(f"Planned scenarios loaded: {len(set(planned))}")

sampled_vars = {
      "scenario": [],
      "t": [],
      "d": [],
      "lon_velocity": [],
}

conditioned_vars = {
      "scenario": [],
      "init_x": [],
      "init_y": [],
      "init_theta": [],
      "init_velocity": [],
      "goal_x": [],
      "goal_y": [],
}

for sc in tqdm(os.listdir(dir), desc="Planning scenarios", unit="scenario"):
      if sc.endswith(".xml"):            
            # if sc in planned:
            #       # print(f"Scenario {sc} already planned, skipping...")
            #       continue
            # else:
            #       print(f"Planning scenario {sc}...") 
            #       planned = np.append(planned, sc)
            #       np.save("CVAE/data/planned_scenarios.npy", planned)

            # temporary storage for sampled variables
            tmp_sampled = {
                  "t": [],
                  "d": [],
                  "lon_velocity": [],
            }
            
            # temporary storage for conditioned variables
            tmp_conditioned = {
                  "init_x": [],
                  "init_y": [],
                  "init_theta": [],
                  "init_velocity": [],
                  "goal_x": [],
                  "goal_y": [],
            }
            
            
            img_save_dir = f"/home/kareem/frenet_optimal_trajectory_planner/CVAE/data/scenarios_imgs/{sc[:-4]}"
            # sc_path = os.path.join(dir, sc)
            
            print(f"Planning for {sc}")
            
            config = ReactivePlannerConfiguration.load(config_file, sc)
            config.update()

            try:
                  # run route planner
                  route_planner = RoutePlanner(config.scenario, config.planning_problem)
                  route = route_planner.plan_routes().retrieve_first_route()
                  
                  # Scenario has attribute position as a lanelet in goal (get the center of the lanelet) 
                  if isinstance(config.planning_problem.goal.state_list[0].position, \
                        commonroad.geometry.shape.ShapeGroup):
                        goal_pos = config.planning_problem.goal.state_list[0].position.shapes[0].center
                        
                  # Scenario has attribute position as a rectangle in goal (get the center of the rectangle)
                  elif isinstance(config.planning_problem.goal.state_list[0].position, \
                        commonroad.geometry.shape.Rectangle):
                        goal_pos = config.planning_problem.goal.state_list[0].position.center
                        
                  # get reference path
                  reference_path = route.reference_path
                  
                  planner = ReactivePlanner(config=config)

                  # set reference path for curvilinear coordinate system
                  planner.set_reference_path(route.reference_path)
                  while not planner.goal_reached():

                        planner.set_desired_velocity(current_speed=planner.x_0.velocity)

                        # call plan function
                        optimal, samples = planner.plan()

                        # record planned state and input
                        planner.record_state_and_input(optimal[0].state_list[1])

                        # reset planner state for re-planning
                        planner.reset(initial_state_cart=planner.record_state_list[-1], 
                                    initial_state_curv=(optimal[2][1], optimal[3][1]),
                                    collision_checker=planner.collision_checker, 
                                    coordinate_system=planner.coordinate_system)
                        
                        # save_scenario_at_timestep(sc[:-4], planner.record_state_list[-1].time_step)
                        
                        # sampled_vars["scenario"].append(sc[:-4])
                        tmp_sampled["t"].append(samples[0])
                        tmp_sampled["d"].append(samples[1])
                        tmp_sampled["lon_velocity"].append(samples[2])
                        
                        # conditioned_vars["scenario"].append(sc[:-4])
                        tmp_conditioned["init_x"].append(config.planning_problem.initial_state.position[0])
                        tmp_conditioned["init_y"].append(config.planning_problem.initial_state.position[1])
                        tmp_conditioned["init_theta"].append(config.planning_problem.initial_state.orientation)
                        tmp_conditioned["init_velocity"].append(config.planning_problem.initial_state.velocity)
                        tmp_conditioned["goal_x"].append(goal_pos[0])
                        tmp_conditioned["goal_y"].append(goal_pos[1])
                  
                  # save sampled variables and conditiobned variables if scenario is successfully planned
                  if planner.goal_reached():
                        print(f"Scenario {sc} successfully planned!")
                        # print(f"Number of samples: {planner.record_state_list[-1].time_step}")
                        save_scenario_imgs(sc[:-4], planner.record_state_list[-1].time_step)
                        
                        for t, d, lon_v in zip(tmp_sampled["t"], tmp_sampled["d"], tmp_sampled["lon_velocity"]):
                              sampled_vars["scenario"].append(sc[:-4])
                              sampled_vars["t"].append(t)
                              sampled_vars["d"].append(d)
                              sampled_vars["lon_velocity"].append(lon_v)
                              
                        for init_x, init_y, init_theta, init_velocity, goal_x, goal_y in zip(
                              tmp_conditioned["init_x"], 
                              tmp_conditioned["init_y"], 
                              tmp_conditioned["init_theta"], 
                              tmp_conditioned["init_velocity"], 
                              tmp_conditioned["goal_x"], 
                              tmp_conditioned["goal_y"]):
                              
                              conditioned_vars["scenario"].append(sc[:-4])
                              conditioned_vars["init_x"].append(init_x)
                              conditioned_vars["init_y"].append(init_y)
                              conditioned_vars["init_theta"].append(init_theta)
                              conditioned_vars["init_velocity"].append(init_velocity)
                              conditioned_vars["goal_x"].append(goal_x)
                              conditioned_vars["goal_y"].append(goal_y)

            except Exception as e:
                  print(f"Scenario {sc} failed!")

                  continue
            
sampled_vars_df = pd.DataFrame(sampled_vars)
sampled_vars_df.to_csv('CVAE/data/sampled_vars.csv')
            
conditioned_vars_df = pd.DataFrame(conditioned_vars)
conditioned_vars_df.to_csv('CVAE/data/conditioned_vars.csv')
                        
            