#!/bin/sh

echo "\n\n\n"
echo "####################################################################################################"
echo "##### Installing gazebo ros packages (http://www.gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages)"
echo "####################################################################################################"



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing gazebo ros packages for ROS Hydro"
echo "----------------------------------------------------"
sudo apt-get install ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control -y


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing dependencies"
echo "----------------------------------------------------"
cd ~/catkin_ws/src
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro hydro
rosdep install --from-paths . --ignore-src --rosdistro hydro -y



echo "\n\n\n"
echo "####################################################################################################"
echo "##### Usage (http://gazebosim.org/wiki/Tutorials/1.9/Using_roslaunch_Files_to_Spawn_Models)"
echo "####################################################################################################"

echo "# Gazebo 1.5: roslaunch gazebo_worlds empty_world.launch"
echo "# Gazebo 1.9: roslaunch gazebo_ros empty_world.launch"
