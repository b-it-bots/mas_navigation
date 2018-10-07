# mcr_move_base_tools

## PlannerUpdater

Provides a functionality to switch global and/or local planner on the fly (when a robot is moving). In the constructor of the class, the PlannerUpdater will search into the plugins available on the system, therefore, just available planners could be called. It can be used via command-line or instantiating the Class with mode selection parameters.

The packages provides two test scripts:

1. update_planner.py -> provides a bash interface to select the planners to update from a list.
2. mode_update_planner.py -> provides a mode-wise interface.

Class Constructor Params:

1. navigation_server: Navigation manager server name, default "/move_base/".
2. config_package: set the package where planners config fiels are located, default "mcr_move_base_tools".
3. config_folder: set the folder inside the config_package where the planners config files are located, default "config".
4. cfg_file: set the path to the file where the cfg inside the cfg_package where the modes and plugins query are defined, default "ros/config/default.yaml").

Just Needed if mode selection is requested.
4. mode_request: name of requested mode, default None.
5. mode_package: set the package where mode config file is located, default "mcr_move_base_tools".

Assumptions:

PlanerUpdater looks for a file named as the namespace of the selected planner. For instance, dwa_local_planner/DWALocalPlanner config file is named dwa_local_planner.yaml.
