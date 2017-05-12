About
-----

This provides experimental repository of ROS support for Choreonoid 

choreonoid\_ros\_pkg: Meta-package to build Choreonoid packages

choreonoid\_ros: Choreonoid catkin package

choreonoid\_plugin: Choreonoid plugins to publish ROS topic

jvrc\_models: Simulation models and tasks for JVRC

Usage
-----

Make sure you have installed wstool and catkin

```
$ sudo apt-get install python-rosinstall python-catkin-tools
```

Create catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
```

Checkout choreonoid\_ros\_pkg

```
$ cd ~/catkin_ws/src
$ wstool init
$ wstool set choreonoid_ros_pkg https://github.com/fkanehiro/choreonoid_ros_pkg.git --git -y
$ wstool update choreonoid_ros_pkg
```

Build

```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
$ export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/$ROS_DISTRO
$ catkin build choreonoid_ros_pkg
$ source install/setup.bash
```

Run

```
$ roslaunch choreonoid_ros choreonoid.launch
```

Note
-----

For further details, please visit https://fkanehiro.github.io/choreonoid_ros_pkg_doc
