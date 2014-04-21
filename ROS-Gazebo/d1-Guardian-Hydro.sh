#!/bin/sh

echo "####################################################################################################"
echo "##### Installing guardian-ros-pkg"
echo "####################################################################################################"


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing dependencies"
echo "----------------------------------------------------"
sudo apt-get install ros-hydro-pr2-common -y
sudo apt-get install ros-hydro-joy -y
sudo apt-get install ros-hydro-joystick-drivers -y
sudo apt-get install ros-hydro-ros-control -y
sudo apt-get install ros-hydro-ros-controllers -y


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Cloning guardian-ros-pkg groovy-devel"
echo "----------------------------------------------------"
cd ~/catkin_ws/src
git clone https://github.com/inesc/guardian-ros-pkg.git -b groovy-devel



echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Building catkin workspace"
echo "----------------------------------------------------"
cd ~/catkin_ws/
catkin_make
