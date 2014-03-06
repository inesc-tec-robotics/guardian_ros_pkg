/*
 * Software License Agreement (BSD License)
 *
 *  modbus_io
 *  Copyright (c) 2012, Robotnik Automation, SLL
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <boost/format.hpp>
#include <iostream>
#include "modbus_io/modbus.h"
#include <stdio.h>

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "modbus_io/modbus_io_msg.h"
#include "modbus_io/write_digital_output.h"
#include "modbus_io/write_analog_output.h"

using namespace std;

class modbusNode
{
public:

  modbus_io::modbus_io_msg reading;  // declare reading message
  string ip_address;

  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher modbus_io_data_pub_;
  ros::ServiceServer modbus_io_write_digital_pub_;
  ros::ServiceServer modbus_io_write_analog_pub_;

  bool running;

  // Error counters and flags
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;

  // Modbus member variables
  modbus_t *mb_;
  uint16_t tab_reg_[32];
  uint16_t do384, do385;  // store digital output registers to activate each one separatedly

  modbusNode(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0),
  slow_count_(0),
  desired_freq_(20),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    running = false;
    ros::NodeHandle modbus_node_handle(node_handle_, "modbus_io");
    private_node_handle_.param("ip_address", ip_address, string("192.168.1.185"));
    modbus_io_data_pub_ = modbus_node_handle.advertise<modbus_io::modbus_io_msg>("input_output", 100);
    modbus_io_write_digital_pub_ = modbus_node_handle.advertiseService("write_digital_output", &modbusNode::write_digital_output, this);
    modbus_io_write_analog_pub_ = modbus_node_handle.advertiseService("write_analog_output", &modbusNode::write_analog_output, this);
    self_test_.add("Connect Test", this, &modbusNode::ConnectTest);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( "Device Status", this, &modbusNode::deviceStatus );
  }

  ~modbusNode()
  {
    stop();
  }

  int start()
  {
    stop();

    mb_=modbus_new_tcp(ip_address.c_str(), 502);

    if (modbus_connect(mb_)== -1){
      ROS_ERROR ("modbus_io::start - Connection Error!");
      return -1;
      }

    ROS_INFO("Connected to MODBUS IO BOARD at %s", ip_address.c_str() );
    freq_diag_.clear();

    running = true;

    return(0);
  }

  int stop()
  {
    if(running)
    {
      modbus_close(mb_);
      modbus_free(mb_);
      running = false;
    }
    return(0);
  }

  int read_and_publish()
  {
      static double prevtime = 0;

      double starttime = ros::Time::now().toSec();
      if (prevtime && prevtime - starttime > 0.05)
      {
        ROS_WARN("Full modbus_io loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
        was_slow_ = "Full modbus_io loop was slow.";
        slow_count_++;
      }

      getData(reading);

      double endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full modbus_io loop was slow.";
        slow_count_++;
      }
      prevtime = starttime;
      starttime = ros::Time::now().toSec();
      modbus_io_data_pub_.publish(reading);

      endtime = ros::Time::now().toSec();
      if (endtime - starttime > 0.05)
      {
        ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
        was_slow_ = "Full modbus_io loop was slow.";
        slow_count_++;
      }

      freq_diag_.tick();
      return(0);
  }

  bool spin()
  {
    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {

      if (start() == 0)
      {
        while(node_handle_.ok()) {
          if(read_and_publish() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
        }
       } else {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        usleep(1000000);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    ROS_INFO("modbus_io::spin - calling stop !");
    stop();
    return true;
  }

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // connection test
    // TBC
    status.summary(0, "Connected successfully.");
  }

  void getData(modbus_io::modbus_io_msg& data)
  {
	// Adress	 Description
	// 0000 	 1 to 8 inputs module IL ETH
	// 0384 	 1 to 4 outuputs module IL ETH 
	// 0385 	 5 to 8 outputs module IB IL DO4
 	// 0192	 	 Analog Input 1
	// 0193 	 Analog Input 2
 	// 0576	 	 Configuration Analog Input 1 
 	// 0577	 	 Configuration Analog Input 2 
	// 0578	 	 Analog Output 1
 	// 0579	  	 Analog Output 2 

	// Read digital inputs
        modbus_read_registers(mb_, 0, 1, tab_reg_);
        //ROS_INFO("modbus_io::getData - Read %d ",tab_reg_[0]);	
	int16_t x = tab_reg_[0];
	for (int i=0; i<8; i++) {
      		reading.digital_input[i] = x&1;
    		x>>=1; 
		}

	// Read digital outputs
	modbus_read_registers(mb_, 384, 2, tab_reg_);
	do384 = x = tab_reg_[0];
	for (int i=0; i<4; i++) {
      		reading.digital_output[i] = x&1;
    		x>>=1; 
		}
	//modbus_read_registers(mb_, 385, 1, tab_reg_);
	do385 = x = tab_reg_[1];
	for (int i=0; i<4; i++) {
      		reading.digital_output[i+4] = x&1;
    		x>>=1; 
		}

	// Read analog inputs 
	modbus_read_registers(mb_, 192, 2, tab_reg_);
	reading.analog_input[0] = double (tab_reg_[0] / 30000.0) * 10.0;  // Range 0-10V
	reading.analog_input[1] = double (tab_reg_[1] / 30000.0) * 10.0;  // Range 0-10V

	// Read analog outputs
	modbus_read_registers(mb_, 578, 2, tab_reg_);
	reading.analog_output[0] = double (tab_reg_[0] / 30000.0) * 10.0;  // Range 0-10V
	reading.analog_output[1] = double (tab_reg_[1] / 30000.0) * 10.0;  // Range 0-10V

        //ROS_INFO("modbus_io::getData - Read %d ",tab_reg_[0]);	
  }
	
  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
  {
    if (!running)
      status.summary(2, "modbus_io is stopped");
    else if (!was_slow_.empty())
    {
      status.summary(1, "Excessive delay");
      was_slow_.clear();
    }
    else
      status.summary(0, "modbus_io is running");

    status.add("Error count", error_count_);
    status.add("Excessive delay", slow_count_);
  }

  // Service write digital output
  bool write_digital_output(modbus_io::write_digital_output::Request &req,
             modbus_io::write_digital_output::Response &res )
  {
    if ((req.output < 0) || (req.output > 7)) {
	res.error = -1;
        return false;
	}
    int iret;
    uint16_t reg_val, bit;
    if (req.output<4) {// 384
	bit = (uint16_t) 1<<req.output;  //
	if (req.value) reg_val = do384 | bit;
	else reg_val = do384 & ~bit;
	iret=modbus_write_register(mb_,384, reg_val);
	}
    else {
	bit = (uint16_t) 1<<(req.output-4);  //
	if (req.value) reg_val = do385 | bit;
	else reg_val = do385 & ~bit;
	iret=modbus_write_register(mb_,385, reg_val);
	}
    ROS_INFO("modbus_io::write_digital_output service request: output=%d, value=%d", (int) req.output, (int)req.value );
    if (iret < 0) res.error = -1; 
    else res.error = 0;
    return (res.error==0);
  }

  // Service write analog output
  bool write_analog_output(modbus_io::write_analog_output::Request &req,
             modbus_io::write_analog_output::Response &res )
  {
    if ((req.output < 0) || (req.output > 1)) {
	res.error = -1;
        return false;
	}
    if (req.value < 0.0) req.value = 0.0;
    if (req.value > 10.0) req.value = 10.0;
    uint16_t value = (uint16_t) ((req.value / 10.0) * 30000.0);
    int iret = modbus_write_register(mb_,578+req.output, value);
    ROS_INFO("request: output=%d, value=%5.4f", (int) req.output, req.value );
    if (iret < 0) res.error = -1; 
    else res.error = 0;
    return (res.error == 0);
  }

};

int 
main(int argc, char** argv)
{
  ros::init(argc, argv, "modbus_io_node");

  ros::NodeHandle nh;

  modbusNode mn(nh);
  mn.spin();

  return(0);
}
