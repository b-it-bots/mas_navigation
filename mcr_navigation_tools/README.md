# mcr_navigation_tools

## Introduction

This package provides different nodes which provides additional features related for navigation.

## navigation_goals_as_markers_node

### Scripts
1. navigation_goals_as_marker: Publish a Marker topic with navigation_goals poses file.

### Published Topic

* /visualization_marker_array : MarkerArray Topic

### Launch Files
1. navigationn_goals_as_marker.launch: Launch file for navigation_goals_as_marker node.

## path_length_calculator_node

### Scripts
1. path_length_calculator_node: This node provides listens to nav_msgs Path topic (which contains a global plan for the mobile base) as an array of poses and calculates the path lenght based on the distance between two points of each pose.

### Subscribed Topics
* "event_in": Start/Stop node
* "plan": Calculate length of this plan.

### Published Topics
* "event_out": Succes/Failure string output
* "path_length": Gives length of the path published on plan

### Launch Files
1. path_length_calculator.launch: Launch path_length_calculator_node.  

## pose_array_to_path
1. pose_array_to_path_node: Subscribes to pose array topic, republishes as nav_msgs/Path topic.

### Subscribed Topics
* "pose_array": PoseArray input topic.

### Published Topics
* "path": Path output topic.

## save_base_map_poses_to_file

### Scripts
1. save_base_map_poses_to_file: Used to create a file in the current folded which contains points of interest in a given map.  

## Arguments

* The default created file is named "navigation_goals.yaml". If a name is provided as an argument, then file name is <provided_name>.yaml 

### Parameters
* node_frequency: Frecuency of node
* "~plan" : set the global_plan_topic to be calculated.

1. pose_array_to_path_converter.launch

### Parameters
* node_frequency: Frecuency of node.
* "~pose_array": Pose Array subscribed topic.
* "~path": Path published topic name.

## map_saver

1. map_saver: Stores a map on the path of the package mcr_default_env_config using the node map_saver from map_server package.

### Arguments 

The map name is introduced on the commandline once the script is running.


