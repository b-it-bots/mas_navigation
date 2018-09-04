import os
from dynamic_reconfigure.client import Client
from mcr_move_base_tools.tools import PlannerUpdater

desired_mode = raw_input('Type desired_mode: ')
planner_updater = PlannerUpdater(mode_request=desired_mode)
planner_updater.update_planners(new_config, new_global_planner_ns=global_planner_ns, new_local_planner_ns=local_planner_ns)
