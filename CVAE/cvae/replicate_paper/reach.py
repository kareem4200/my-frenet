from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual

import matplotlib
matplotlib.use('Agg')  # Non-GUI backend (renders to image files)



# build configuration
# make sure that the "scenarios" directory in your current directory contains the ARG_Carcarana-1_1_T-1 scenario
config = ConfigurationBuilder().build_configuration("USA_Lanker-2_18_T-1")
config.update()

# compute reachable sets using reachability interface
reach_interface = ReachableSetInterface(config)
reach_interface.compute_reachable_sets()

# plot computation results
util_visual.plot_scenario_with_reachable_sets(reach_interface)
for i in range(80):
      print(len(reach_interface.drivable_area_at_step(step=i)))
      print(reach_interface.drivable_area_at_step(step=i))
# print(reach_interface.reachable_set_at_step(step=20))