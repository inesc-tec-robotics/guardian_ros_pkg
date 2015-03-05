Robotnik Guardian - ROS and Gazebo packages


# 1. Setup:
ROS Hydro + Gazebo 1.9

* http://wiki.ros.org/hydro/Installation/Ubuntu
* http://gazebosim.org/tutorials?tut=install&ver=1.9

# 2. Dependencies:

## install these deps:
* sudo apt-get install ros-hydro-pr2-common
* sudo apt-get install ros-hydro-joy
* sudo apt-get install ros-hydro-joystick-drivers
* sudo apt-get install ros-hydro-ros-control
* sudo apt-get install ros-hydro-ros-controllers
* sudo apt-get install ros-hydro-controller-manager
* sudo apt-get install ros-hydro-gazebo-ros-control
* sudo apt-get install ros-hydro-gazebo-plugins
* sudo apt-get install ros-hydro-hector-gazebo-plugins

# 3. To launch the simulation:

## Guardian in empty world:
roslaunch guardian_gazebo guardian.launch

## Guardian inside ship:
roslaunch guardian_gazebo guardian_ship_interior.launch

### These also launch joystick and keyboard nodes to control the robot

* keyboard: wasd keys

* joystick: config files @ guardian_joystick/launch/


		  if needed create yout own and alter the line 
		  	<rosparam command="load" file="$(find guardian_joystick)/launch/logitech.yaml" />
		  to
		  	<rosparam command="load" file="$(find guardian_joystick)/launch/MyOwnConfig.yaml" />