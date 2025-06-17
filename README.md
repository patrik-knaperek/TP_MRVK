# MRVK ROS Workspace

## Installation requirements

**ROS packages**
* image_view
* gmapping
* gazebo_ros_packages
* costmap-2d

Install with command
```sh
$ sudo apt install ros-<distro>-<package_name1> ros-<distro>-<package_name2> ...
```

## Install

```sh
$ cd ${ROS_WS}
$ catkin_make
$ source ./devel/setup.bash
$ export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```

## Launch simulation

In file [robot.launch](./mrvk_gazebo/launch/robot.launch) choose the world to be launched (uncomment or rewrite).

```sh
$ roslaunch mrvk_gazebo robot.launch
```
### Launch Path Detection

```sh
$ rosrun path_detection path_detection_node.py
```
