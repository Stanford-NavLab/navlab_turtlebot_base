# navlab_turtlebot_base

ROS package for working with [TurtleBot](https://www.robotis.us/turtlebot-3/) robots
## Install Instructions

1. Install dependencies:
```
sudo apt-get install ros-"${ROS_DISTRO}"-turtlebot3 ros-"${ROS_DISTRO}"-vrpn
```

## Gazebo Simulation w/ Move Base and Goal Planner

Simulate flight room in Gazebo environment:
```
roslaunch navlab_turtlebot_bringup navigate_multi.launch planner:=goal
```

## Stanford Flight Room Launch Instructions w/ Move Base and Goal Planner
This section contains specific details for running in the [Stanford Flight Room](https://stanfordflightroom.github.io/).

On the motion capture relay computer run:
```
roscore
```

On each turtlebot run
```
roslaunch navlab_turtlebot_bringup move_base_multi.launch
```

On the base/groundstation computer run:
```
roslaunch navlab_turtlebot_base navigate_multi.launch planner:=goal sim:=false
```
