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

planner_updater.deleteOldParams()


#TODO FROM HERE ON

print "LOADING NEW PARAMS GLOBAL PLANNER"

new_global_planner_ns = new_config["base_global_planner"].split('/')[0]
print config_path+'/'+ new_global_planner_ns + '.yaml'

new_config_file = rosparam.load_file(config_path+'/'+ new_global_planner_ns + '.yaml')

for params,ns in new_config_file:
    print ns, params
    rosparam.upload_params(navigation_server + ns,params)


print "LOADING NEW PARAMS LOCAL PLANNER"

new_local_planner_ns = new_config["base_local_planner"].split('/')[0]
print config_path+'/'+ new_local_planner_ns + '.yaml'

new_config_file = rosparam.load_file(config_path+'/'+ new_local_planner_ns + '.yaml')

for params,ns in new_config_file:
    print ns, params
    rosparam.upload_params(navigation_server + ns,params)


print new_config
dyn_client.update_configuration(new_config)
#break

print "finish"
