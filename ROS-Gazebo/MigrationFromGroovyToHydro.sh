#!/bin/sh

echo "####################################################################################################"
echo "##### Migrating from ROS Groovy to ROS Hydro"
echo "####################################################################################################"


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing ROS Hydro"
echo "------------------------------------------------"
sudo apt-get install ros-hydro-desktop-full -y --force-yes



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Initializing rosdep"
echo "------------------------------------------------"
sudo rm -rf /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up environment"
echo ">>>>> Warning: Comment groovy sources and exports in ~/.bashrc"
echo "------------------------------------------------"
echo "# <ROS hydro setup.bash>" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc
export ROBOT=sim
echo "export ROS_ENV_LOADER=/etc/ros/hydro/env.sh" >> ~/.bashrc
export ROS_ENV_LOADER=/etc/ros/hydro/env.sh
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source /opt/ros/hydro/setup.bash
echo "# </ROS hydro setup.bash>" >> ~/.bashrc



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Creating new catkin workspace"
echo "------------------------------------------------"
cd ~/
mv catkin_ws catkin_ws_groovy
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws/
catkin_make



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing remaining packages"
echo "------------------------------------------------"
sudo apt-get install ros-hydro-pcl-* -y
sudo apt-get install ros-hydro-openni-* -y --force-yes
sudo apt-get install ros-hydro-gazebo-ros-pkgs ros-hydro-gazebo-ros-control -y --force-yes