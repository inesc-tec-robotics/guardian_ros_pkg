cmake_minimum_required(VERSION 2.8.3)
project(guardian_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_srvs
    std_msgs
    tf
    gazebo_ros
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS 
    roscpp 
    gazebo_ros 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(beginner_tutorials
#   src/${PROJECT_NAME}/beginner_tutorials.cpp
# )

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(gazebo_ros_laser_2D SHARED src/laser_2D.cpp)
target_link_libraries(gazebo_ros_laser_2D ${catkin_LIBRARIES} ${GAZEBO_libraries} RayPlugin)

add_library(skid_steer_plugin SHARED src/skid_steer_plugin.cpp)
target_link_libraries(skid_steer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


