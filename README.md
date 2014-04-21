Installation scripts
================

The scripts in folder ROS-Gazebo aim to ease the setup of guardian-ros-pkg  
They should be used in alphanumeric order (some are optional).
They were tested in Ubuntu 12.04 LTS with ROS Groovy and Hydro.


* **_[a1-Apps.sh (optional)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/a1-Apps.sh)_**
  * Script to install some of the most used packages after a clean install of a Ubuntu distribution
* **_[a2-GitConfigs.sh (optional)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/a2-GitConfigs.sh)_**
  * Script to setup .gitconfig
* **_[b1-ROS-Groovy.sh (required for developing in ROS Groovy)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b1-ROS-Groovy.sh)_**
  * Script to install ROS Groovy (customize the first section to your Ubuntu distribution)
* **_[b1-ROS-Hydro.sh (required for developing in ROS Hydro)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b1-ROS-Hydro.sh)_**
  * Script to install ROS Hydro (customize the first section to your Ubuntu distribution)
* **_[b2-CatkinWorkspace.sh (required)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b2-CatkinWorkspace.sh)_**
  * Script to setup catkin workspace
* **_[b3-Gazebo.sh (required)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b3-Gazebo.sh)_**
  * Script to install Gazebo simulator (customize the first section to your Ubuntu distribution)
* **_[b4-Gazebo-ros-pkgs-Groovy.sh (required for developing in ROS Groovy)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b4-Gazebo-ros-pkgs-Groovy.sh)_**
  * Script to integrate Gazebo with ROS Groovy (required for developing in ROS Groovy)
* **_[b4-Gazebo-ros-pkgs-Hydro.sh (required for developing in ROS Hydro)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/b4-Gazebo-ros-pkgs-Hydro.sh)_**
  * Script to integrate Gazebo with ROS Hydro (required for developing in ROS Hydro)
* **_[c1-RViz-catkin-Groovy.sh (optional, but recommended for ROS Groovy)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/c1-RViz-catkin-Groovy.sh)_**
  * Script to update RViz to latest Groovy version
* **_[c1-RViz-rosbuild-Groovy.sh (optional for Groovy)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/c1-RViz-rosbuild-Groovy.sh)_**
  * Script equivalent to c1-RViz-catkin-Groovy.sh but without using catkin
* **_[d1-Guardian-Groovy.sh (required for developing in ROS Groovy)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/d1-Guardian-Groovy.sh)_**
  * Script to setup the guardian-ros-pkg from the inesc repository for ROS Groovy (fork of https://github.com/RobotnikES/guardian-ros-pkg)
* **_[d1-Guardian-Hydro.sh (required for developing in ROS Hydro)](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/d1-Guardian-Hydro.sh)_**
  * Script to setup the guardian-ros-pkg from the inesc repository for ROS Hydro (fork of https://github.com/RobotnikES/guardian-ros-pkg)
* **_[MigrationFromGroovyToHydro.sh](https://github.com/inesc/guardian-ros-pkg/blob/configs/ROS-Gazebo/MigrationFromGroovyToHydro.sh)_**
  * Script to ease the transition from ROS Groovy to ROS Hydro



Eclipse integration with ROS
================ 
  
[Eclipse](http://www.eclipse.org/downloads/) is an IDE capable to significantly increase the productivity of any programmer in almost all the know languages and formats.
For ROS developers is useful to debug nodes, profiling, code completion, documentation, testing...


* **_[eclipsePlugins.txt](https://github.com/inesc/guardian-ros-pkg/blob/configs/Eclipse/eclipsePlugins.txt)_**
  * Instructions to setup Eclipse CDT with useful plugins for ROS development
* **_[rosWithEclipse.txt](https://github.com/inesc/guardian-ros-pkg/blob/configs/Eclipse/rosWithEclipse.txt)_**
  * Instructions to integrate Eclipse with ROS (for debugging, using with catkin / rosbuild)
* **_[rosWithEclipse.pptx](https://github.com/inesc/guardian-ros-pkg/raw/configs/Eclipse/rosWithEclipse.pptx)_**
  * Presentation with screenshots showing how to integrate Eclipse with ROS
* **_[eclipseShortcuts.txt](https://github.com/inesc/guardian-ros-pkg/blob/configs/Eclipse/eclipseShortcuts.txt)_**
  * Useful shortcuts to increase productivity in Eclipse
