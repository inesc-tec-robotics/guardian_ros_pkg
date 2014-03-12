cmake_minimum_required(VERSION 2.8.3)
project(modbus_io)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    message_generation
    roscpp 
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
    )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   modbus_io_msg.msg  
)

## Generate services in the 'srv' folder
add_service_files( DIRECTORY srv 
    FILES
    write_analog_output.srv  
    write_digital_output.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   robotnik_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES summit_xl_robot_control
  CATKIN_DEPENDS 
    roscpp 
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(modbus_io_node src/modbus_io_node.cpp src/modbus-data.c src/modbus-rtu.c src/modbus-tcp.c src/modbus.c)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(modbus_io_node modbus_io_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(modbus_io_node
   ${catkin_LIBRARIES}
 )
