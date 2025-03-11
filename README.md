# MRVK ROS Workspace

## Installation requirements

**ROS packages**
* image_view
* gmapping
* gazebo_ros_packages
* costmap-2d

## Install

```sh
$ cd ${ROS_WS}
$ catkin_make
$ source ./devel/setup.bash
```

## Launch simulation

In file [robot.launch](./mrvk_gazebo/launch/robot.launch) choose the world to be launched (uncomment or rewrite).

## Before launching bayland world you need to update gazebo model database

```sh
$ export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```