[![Build Status](https://travis-ci.org/b-it-bots/mas_navigation.svg?branch=kinetic)](https://travis-ci.org/b-it-bots/mas_navigation)

# mas_navigation

## Getting started

b-it-bots members can use [these instructions](https://github.com/b-it-bots/dev-env#setup) to setup a complete development environment for all our robots.

For external users, the following instructions should get you a working system:

1. Setup a catkin workspace

  ```
  mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
  wstool init src
  wstool merge -t src https://raw.githubusercontent.com/b-it-bots/mas_navigation/kinetic/mas-navigation.rosinstall
  ```
2. Get the code and dependencies

  ```
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

  ```

3. Building your code

  ```
  catkin build
  ```
