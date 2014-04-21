#!/bin/sh

echo "####################################################################################################"
echo "##### Installing guardian-ros-pkg"
echo "####################################################################################################"


echo "\n\n"
echo "----------------------------------------------------"
echo ">>>>> Installing dependencies"
echo "----------------------------------------------------"
sudo apt-get install ros-groovy-pr2-common -y
sudo apt-get install ros-groovy-joy -y
sudo apt-get install ros-groovy-joystick-drivers -y
sudo apt-get install ros-groovy-ros-control -y
sudo apt-get install ros-groovy-ros-controllers -y


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
