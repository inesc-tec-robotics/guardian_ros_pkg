cmake_minimum_required(VERSION 2.8.3)
project(modbus_io)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	self_test
	diagnostic_updater
	std_srvs
	message_generation
)

add_message_files(
  FILES
  modbus_io_msg.msg
)

add_service_files(DIRECTORY srv 
  FILES
  write_analog_output.srv
  write_digital_output.srv
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS
    roscpp
    self_test
    diagnostic_updater
    std_srvs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(modbus_io src/modbus_io_node.cpp src/modbus.c src/modbus-data.c src/modbus-rtu.c src/modbus-tcp.c)
target_link_libraries(modbus_io ${catkin_LIBRARIES})
