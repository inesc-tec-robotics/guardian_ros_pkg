guardian-ros-pkg
========

Robotnik Guardian - ROS and Gazebo packages


# 1. Setup:
ROS Hydro + Gazebo 1.9

# 2. Dependencies:

## install these deps:
sudo apt-get install ros-hydro-pr2-common
sudo apt-get install ros-hydro-joy
sudo apt-get install ros-hydro-joystick-drivers
sudo apt-get install ros-hydro-ros-control
sudo apt-get install ros-hydro-ros-controllers
sudo apt-get install ros-hydro-controller-manager

## clone these repos to your workspace:
git clone https://github.com/inesc-tec-robotics/crob_gazebo_models


# 3. To launch the simulation:

## Guardian in empty world:
roslaunch guardian_gazebo guardian.launch

## Guardian inside ship:
roslaunch guardian_gazebo guardian_ship_interior.launch 
