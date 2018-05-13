This component moves the mobile base in Cartesian space until a pose is reached.

### Usage
1. Publish the pose that the robot should reach on the topic `/mcr_navigation/direct_base_controller/input_pose` in a static frame (such as odom)
2. Publish `e_start` on the topic `/mcr_navigation/direct_base_controller/coordinator/event_in`
3. Monitor the topic `/mcr_navigation/direct_base_controller/coordinator/event_out`
