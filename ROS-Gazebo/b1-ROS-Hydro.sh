#!/bin/sh

echo "####################################################################################################"
echo "##### Installing ROS Groovy (http://wiki.ros.org/hydro/Installation/Ubuntu)"
echo "####################################################################################################"


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up sources.list"
echo "------------------------------------------------"

# >>> Ubuntu 12.04 (Precise)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'

# >>> Ubuntu 12.10 (Quantal)
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list'

# >>> Ubuntu 13.04 (Raring)
# sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting keys"
echo "------------------------------------------------"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Updating packages index"
echo "------------------------------------------------"
sudo apt-get update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing ROS"
echo "------------------------------------------------"
# >>> Desktop-Full Install (Recommended)
sudo apt-get install ros-hydro-desktop-full -y --force-yes

# >>> Desktop Install
# sudo apt-get install ros-hydro-desktop -y

# >>> ROS-Base (Bare Bones)
# sudo apt-get install ros-hydro-ros-base -y



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Initializing rosdep"
echo "------------------------------------------------"
sudo rosdep init
rosdep update



echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Setting up environment"
echo "------------------------------------------------"
echo "# <ROS hydro setup.bash>" >> ~/.bashrc
echo "export ROBOT=sim" >> ~/.bashrc
export ROBOT=sim
echo "export ROS_ENV_LOADER=/etc/ros/hydro/env.sh" >> ~/.bashrc
export ROS_ENV_LOADER=/etc/ros/hydro/env.sh
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
source /opt/ros/hydro/setup.bash
# echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Projects" >> ~/.bashrc
echo "# </ROS hydro setup.bash>" >> ~/.bashrc


echo "\n\n"
echo "------------------------------------------------"
echo ">>>>> Installing remaining packages"
echo "------------------------------------------------"
sudo apt-get install python-rosinstall -y
sudo apt-get install ros-hydro-pcl-* -y
sudo apt-get install ros-hydro-openni-* -y --force-yes
