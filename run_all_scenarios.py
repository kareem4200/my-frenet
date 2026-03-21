import os
import subprocess

# Define the path to the folder containing scenario files
scenario_path = os.getcwd() + '/commonroad_utils/Critical_Transformed/'

# Get all XML files in the folder
scenario_files = [f for f in os.listdir(scenario_path) if f.endswith('.xml')]

# Loop through all scenario files
for scenario_name in scenario_files:
    print(f"Running scenario: {scenario_name}")
    # Call main.py with the scenario_name as a parameter
    subprocess.run(['python', 'main.py', scenario_name])