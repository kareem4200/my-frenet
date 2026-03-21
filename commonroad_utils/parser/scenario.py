import os
from commonroad.common.file_reader import CommonRoadFileReader

def get_scenario(path: str, file_name: str):
      # read in scenario and planning problem set
      scenario, planning_problem_set = CommonRoadFileReader(os.path.join(path, file_name)).open()
      # retrieve the first planning problem in the problem set
      planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
      
      return scenario, planning_problem, planning_problem_set







