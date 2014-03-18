Installation scripts
================

The scripts in folder ROS-Gazebo aim to ease the setup of guardian-ros-pkg  
They should be used in alfanumeric order (some are optional).


* **_a1-Apps.sh (optional)_**
  * Script to install some of the most used packages after a clean install of a Ubuntu distribution
* **_a2-GitConfigs.sh (optional)_**
  * Script to setup git
* **_b1-ROS-Groovy.sh (required)_**
  * Script to install ROS Groovy (customize the first section to your Ubuntu distribution)
* **_b2-CatkinWorkspace.sh (required)_**
  * Script to setup catkin workspace
* **_b3-Gazebo.sh (requried)_**
  * Script to install Gazebo simulator (customize the first section to your Ubuntu distribution)
* **_b4-Gazebo-ros-pkgs.sh (required)_**
  * Script to integrate Gazebo with ROS (required)
* **_c1-RViz-catkin.sh (optonal, but recommended)_**
  * Script to update RViz to latest groovy version
* **_c1-RViz-rosbuild.sh (optional)_**
  * Script equivalent to c1-RViz-catkin.sh but without using catkin
* **_d1-Guardian.sh (required)_**
  * Script to setup the guardian-ros-pkg from the inesc repository (fork of https://github.com/RobotnikES/guardian-ros-pkg)
