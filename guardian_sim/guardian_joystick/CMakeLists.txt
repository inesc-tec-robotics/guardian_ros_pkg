cmake_minimum_required(VERSION 2.8.3)
project(guardian_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs
    message_generation 
    )
    
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED COMPONENTS system thread)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv 
   FILES
    enable_disable.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   nav_msgs robotnik_msgs sensor_msgs
 )

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
#  LIBRARIES 
  CATKIN_DEPENDS 
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
  #DEPENDS system_lib
  DEPENDS thread
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${Boost_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS} 
)



## Declare a cpp executable
add_executable(guardian_joystick src/guardian_joystick.cpp)
add_executable(guardian_keyboard src/guardian_keyboard_teleop.cpp)

#link_boost(guardian_keyboard thread)
#target_link_libraries(guardian_keyboard thread ${catkin_LIBRARIES})
#target_link_boost(guardian_keyboard thread ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(guardian_joystick guardian_joystick_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(guardian_joystick ${catkin_LIBRARIES})
target_link_libraries(guardian_keyboard ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_dependencies(guardian_joystick ${PROJECT_NAME}_generate_messages_cpp)

#target_link_boost(${CORE_LIBRARY_TARGET})

 


