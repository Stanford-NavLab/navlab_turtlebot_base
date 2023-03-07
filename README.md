# navlab_turtlebot_base

ROS package for working with [TurtleBot](https://www.robotis.us/turtlebot-3/) robots
## Install Instructions

1. Install dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-turtlebot3 ros-"${ROS_DISTRO}"-vrpn-client-ros
```
2. Set the turtlebot model environment variable or add to your `.bashrc`: `export TURTLEBOT3_MODEL="burger"`

## Gazebo Simulation

Simulate flight room in Gazebo environment:
```
roslaunch navlab_turtlebot_base sim_flightroom_navigate.launch
```

## Stanford Flight Room Launch Instructions
This section contains specific details for running in the [Stanford Flight Room](https://stanfordflightroom.github.io/).





Flight Room Demo

motion capture relay:
```
roscore
```

turtlebot
```
roslaunch turtlebot3_bringup turtlebot3_core.launch multi_robot_name:=turtlebot_core
```

groundstation
```
roslaunch navlab_turtlebot_base flightroom_single.launch ns:=turtlebot3
```
