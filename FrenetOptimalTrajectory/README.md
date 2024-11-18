

<h3 align="center">Connecting CommonRoad to Frenet C++ Planner Module</h3>


---

<p align="center"> Helper function to parse commonroad scenarios and planning problem to be planned by an external FreNet C++ module.
    <br> 
</p>

## 📝 Directory Layout

- **FrenetOptimalTrajectory/**
  - **scenarios/**: Original (unplanned) scenarios from CommonRoad
    - **. . .**
  - **scenarios_modified/**: Scenarios that were planned succesfully by CommonRoad Reactive Planner and modified to include the ego vehicles as a dynamic obstacles in them
    - **. . .**
  - **config_files/**: Configuration files for running CriMe on the modified scenarios (contain the ego vehicle ID for each scenario)
    - **. . .**
  - **parser/**
    - **parser.py**: Main parser for waypoints, obstacles, positions and velocity
    - **scenario.py**: Reads the scenario and its planning problem
    - **utils.py**: Contains functions for visualizing Frenet's solution and transform this solution to a CommonRoad Trajectory object
  - **critical_analysis/**
    - **Critical_Original/**: Original (unplanned) chosen critical scenarios
    - **Critical_Planned/**: The critical scenarios planned using CommonRoad Reactive Planner
    - **Critical_PP_Modified/**: Contains the original critical scenarios after modification to include a goal state to the planning problem.
    - **Eval_Results/**: Excel sheets for the results of multiple criticality measures
      - **. . .**
    - **frenet_commonroad.ipynb**: Main notebook for solving CommonRoad scenarios using Frenet (make sure to change the scenario_path to your global path and the .so file global path in fot_wrapper.py)
    - **Misc_Functions.ipynb**: Contains code to other miscelaneous function like planning for all scenarios, analyze criticality for sceenarios, etc...
  - **README.md**: README

---