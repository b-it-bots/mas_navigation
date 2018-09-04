import os
from dynamic_reconfigure.client import Client
from mcr_move_base_tools.tools import PlannerUpdater

desired_mode = raw_input('Type desired_mode: ')
planner_updater = PlannerUpdater(mode_request=desired_mode)
