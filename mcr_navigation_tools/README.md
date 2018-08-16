# mcr_navigation_tools

## Introduction

This package provides different nodes which provides additional features related for navigation.

## pose_visualiser

### Scripts

* `pose_visualiser`:
    1. reads poses from a navigation goals yaml file in which poses are represented as `<NAME>: [x, y, theta]` pairs and
    2. publishes two marker arrays: one array with arrow markers and another one with text markers

### Published Topic

* `/poses`: visualization_msgs.MarkerArray
* `/pose_text`: visualization_msgs.MarkerArray

### Launch Files
1. `pose_visualiser.launch`: Launch file for the `pose_visualiser` node. Two parameters can be specified in the launch file:
    * `pose_frame`: Frame in which the poses should be published
    * `pose_description_file`: Absolute path to a yaml file in which poses are represented as `<NAME>: [x, y, theta]` pairs

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


## Laser distances

### Parameters
* "footprint": rectangular footprint of the robot

### Subscribed Topics
* "scan_front" and "scan_rear": laser scan topics

### Published Topics
* "distances": Minimum distance from footprint to obstacle in the front, right, rear and left directions (in that order)
