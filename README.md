About
-----

This is an EXPERIMENTAL repository of ROS support for Choreonoid 

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

Initialize and update the ROS package database 

```
$ sudo rosdep init
$ rosdep update
```

Install simtrans

```
$ sudo add-apt-repository ppa:hrg/daily
$ sudo apt-get update
$ sudo apt-get install python-pip openhrp meshlab imagemagick python-omniorb openrtm-aist-python
$ git clone https://github.com/fkanehiro/simtrans.git
$ cd simtrans
$ sudo pip install -r requirements.txt
$ sudo pip install enum34
$ sudo python setup.py install
```

Create catkin workspace

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
$ catkin config --merge-devel
```

Checkout choreonoid\_ros\_pkg

```
$ cd ~/catkin_ws/src
$ git clone -b exp https://github.com/s-kajita/choreonoid_ros_pkg.git
```

Build

```
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
$ export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/$ROS_DISTRO
$ catkin build choreonoid_ros_pkg
$ source devel/setup.bash
```

Run

```
$ roslaunch choreonoid_ros choreonoid.launch
$ roslaunch choreonoid_ros jvrc-1-rviz.launch 
$ roslaunch choreonoid_ros ros-tank.launch
```

Note
-----

For further details, please visit https://fkanehiro.github.io/choreonoid_ros_pkg_doc

SlideShare https://www.slideshare.net/shuujikajita/choreonoidros
