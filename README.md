# MRVK ROS Workspace

## Installation requirements

**ROS packages**
* image_view
* gmapping
* gazebo_ros_packages

## Install

```sh
$ cd ${ROS_WS}
$ catkin_make
$ source ./devel/setup.bash
```

## Launch simulation

In file [robot.launch](./mrvk_gazebo/launch/robot.launch) choose the world to be launched (uncomment or rewrite).
