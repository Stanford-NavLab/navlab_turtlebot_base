# navlab_turtlebot

Utility functions for working with [TurtleBot](https://www.robotis.us/turtlebot-3/) robots in the [Stanford Flight Room].(https://stanfordflightroom.github.io/)

## Install Instructions

Install dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-dwa-local-planner ros-"${ROS_DISTRO}"-turtlebot3 ros-"${ROS_DISTRO}"-vrpn-client-ros ros-"${ROS_DISTRO}"-move-base
```

## Launch Instructions

Simulate flight room in Gazebo environment:
```
roslaunch navlab_turtlebot sim_flightroom_navigate.launch
```
