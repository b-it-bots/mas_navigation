import os
from dynamic_reconfigure.client import Client
from mcr_move_base_tools.tools import PlannerUpdater

planner_updater = PlannerUpdater()

new_config = dict()
new_config["base_global_planner"] = "Error"
new_config["base_local_planner"] = "Error"

print "Select Your Global Planner"

available_g_planners = planner_updater.getAvailableGlobalPlanners()

for i in range(len(available_g_planners)):
    print "Available GP ", i , available_g_planners[i]

new_config["base_global_planner"] = available_g_planners[int(raw_input('Choose a number: '))]

print "Select Your Local Planner"

available_l_planners = planner_updater.getAvailableLocalPlanners()

for i in range(len(available_l_planners)):
    print "Available LP ", i , available_l_planners[i]

new_config["base_local_planner"] = available_l_planners[int(raw_input('Choose a number: '))]

global_planner_ns = new_config["base_global_planner"].split('/')[0]

local_planner_ns = new_config["base_local_planner"].split('/')[0]

planner_updater.updatePlanner(new_config, new_global_planner_ns=global_planner_ns, new_local_planner_ns=local_planner_ns)
