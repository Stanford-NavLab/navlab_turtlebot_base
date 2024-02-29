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
roslaunch navlab_turtlebot_bringup navigate_multi.launch robot_count:=4 goal_file:=$(rospack find navlab_turtlebot_sim)/param/four_square.yaml planner:=goal odom_source:=mocap
```

## Stanford Flight Room Launch Instructions w/ Move Base and Goal Planner
This section contains specific details for running in the [Stanford Flight Room](https://stanfordflightroom.github.io/).

On the motion capture relay computer run:
```
roscore
```

On each turtlebot run
```
roslaunch navlab_turtlebot_bringup turtlebot_bringup.launch planner:=goal zed:=true odom_source:=mocap
```

On the base/groundstation computer run:
```
roslaunch navlab_turtlebot_bringup navigate_multi.launch robot_count:=4 goal_file:=$(rospack find navlab_turtlebot_sim)/param/four_square.yaml planner:=goal sim:=false odom_source:=mocap

```
