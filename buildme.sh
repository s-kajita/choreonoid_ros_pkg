cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
export CMAKE_PREFIX_PATH=~/catkin_ws/devel:/opt/ros/$ROS_DISTRO
catkin build choreonoid_ros_pkg
