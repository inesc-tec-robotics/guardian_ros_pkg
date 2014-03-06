To test the component:

-Check IP address 

-Run the node with rosrun modbus_io modbus_io_node
or roslaunch modbus_io_node test_io.launch

-View digital and analog input status with
rostopic echo /modbus_io_node/input_output

-Test digital and analog outputs
rosservice call /modbus_io/write_digital_output 3 false
rosservice call /modbus_io/write_digital_output 0 true
rosservice call /modbus_io/write_analog_output 0 5.0
rosservice call /modbus_io/write_analog_output 1 6.0
