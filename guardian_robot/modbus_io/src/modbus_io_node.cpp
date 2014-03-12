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
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/set_analog_output.h>

#define MODBUS_DESIRED_FREQ	10.0

int MODBUS_DEFAULT_DIGITAL_OUTPUTS = 8;
int MODBUS_DEFAULT_DIGITAL_INPUTS  = 8;
int MODBUS_DEFAULT_ANALOG_OUTPUTS  = 2;
int MODBUS_DEFAULT_ANALOG_INPUTS = 2;
int MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS = 4;	// Min. number of digital outputs (factory default)
int MODBUS_DEFAULT_MIN_DIGITAL_INPUTS	= 8;	// Min. number of digital inputs (factory default)
int MODBUS_DEFAULT_MIN_ANALOG_OUTPUTS   = 0;	// Min. number of analog outputs (factory default)
int MODBUS_DEFAULT_MIN_ANALOG_INPUTS	= 0;	// Min. number of analog inputs (factory default)


using namespace std;

class modbusNode
{
public:

	robotnik_msgs::inputs_outputs reading;  // declare reading message
	string ip_address;

	self_test::TestRunner self_test_;
	diagnostic_updater::Updater diagnostic_;

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	ros::Publisher modbus_io_data_pub_;
	ros::ServiceServer modbus_io_write_digital_pub_;
	ros::ServiceServer modbus_io_write_analog_pub_;

	bool running;
	// Config params
	int digital_inputs_, digital_outputs_;
	int analog_inputs_, analog_outputs_;

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
	
	float max_delay;

	modbusNode(ros::NodeHandle h) : self_test_(), diagnostic_(),
	node_handle_(h), private_node_handle_("~"), 
	error_count_(0),
	slow_count_(0),
	desired_freq_(20),
	freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
	{
		running = false;
		ros::NodeHandle modbus_node_handle(node_handle_, "modbus_io");
		// READ PARAMS
		private_node_handle_.param("ip_address", ip_address, string("192.168.1.185"));
		private_node_handle_.param("digital_outputs", digital_outputs_, MODBUS_DEFAULT_DIGITAL_OUTPUTS);
		private_node_handle_.param("digital_inputs", digital_inputs_, 	MODBUS_DEFAULT_DIGITAL_INPUTS);
		private_node_handle_.param("analog_outputs", analog_outputs_, 	MODBUS_DEFAULT_ANALOG_OUTPUTS);
		private_node_handle_.param("analog_inputs", analog_inputs_, 	MODBUS_DEFAULT_ANALOG_INPUTS);
		
		// Checks the min num of digital outputs
		if(digital_outputs_ < MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS){
			digital_outputs_ = MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS;
			ROS_INFO("modbus_io: Setting num of digital outputs to the minimum value = %d", MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS);
		}		
		// Checks the min num of digital inputs
		if(digital_inputs_ < MODBUS_DEFAULT_MIN_DIGITAL_INPUTS){
			digital_inputs_ = MODBUS_DEFAULT_MIN_DIGITAL_INPUTS;
			ROS_INFO("modbus_io: Setting num of digital inputs to the minimum value = %d", MODBUS_DEFAULT_MIN_DIGITAL_INPUTS);
		}
		// Checks the min num of analog outputs
		if(analog_outputs_ < MODBUS_DEFAULT_MIN_ANALOG_OUTPUTS){
			analog_outputs_ = MODBUS_DEFAULT_MIN_ANALOG_OUTPUTS;
			ROS_INFO("modbus_io: Setting num of analog outputs to the minimum value = %d", MODBUS_DEFAULT_MIN_ANALOG_OUTPUTS);
		}		
		// Checks the min num of analog inputs
		if(analog_inputs_ < MODBUS_DEFAULT_MIN_ANALOG_INPUTS){
			analog_inputs_ = MODBUS_DEFAULT_MIN_ANALOG_INPUTS;
			ROS_INFO("modbus_io: Setting num of analog inputs to the minimum value = %d", MODBUS_DEFAULT_MIN_ANALOG_INPUTS);
		}
	
		ROS_INFO("modbus_io: Settings -> DO = %d, DI = %d, AO = %d, AI = %d", digital_outputs_, digital_inputs_, analog_outputs_, analog_inputs_ );
		modbus_io_data_pub_ = modbus_node_handle.advertise<robotnik_msgs::inputs_outputs>("input_output", 100);
		modbus_io_write_digital_pub_ = modbus_node_handle.advertiseService("write_digital_output", &modbusNode::write_digital_output, this);
		modbus_io_write_analog_pub_ = modbus_node_handle.advertiseService("write_analog_output", &modbusNode::write_analog_output, this);
		self_test_.add("Connect Test", this, &modbusNode::ConnectTest);
		diagnostic_.add( freq_diag_ );
		diagnostic_.add( "Device Status", this, &modbusNode::deviceStatus );

		// Initializes the outputs/inputs vector
		reading.digital_inputs.resize(digital_inputs_);
		reading.digital_outputs.resize(digital_outputs_);
		if(analog_inputs_ > 0)
			reading.analog_inputs.resize(analog_inputs_);
		if(analog_outputs_ > 0)	
			reading.analog_outputs.resize(analog_outputs_);
		max_delay = 1.0 / MODBUS_DESIRED_FREQ;
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
		if (endtime - starttime > max_delay)
		{
			ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full modbus_io loop was slow.";
			slow_count_++;
		}
		prevtime = starttime;
		starttime = ros::Time::now().toSec();
		modbus_io_data_pub_.publish(reading);

		endtime = ros::Time::now().toSec();
		if (endtime - starttime > max_delay)
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
		ros::Rate r(MODBUS_DESIRED_FREQ);
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
					r.sleep();
				}
			} else {
				// No need for diagnostic here since a broadcast occurs in start
				// when there is an error.
				sleep(1);
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

	void getData(robotnik_msgs::inputs_outputs& data)
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
			reading.digital_inputs[i] = x&1;
			x>>=1; 
		}

		// Read digital outputs
		modbus_read_registers(mb_, 384, 2, tab_reg_);
		do384 = x = tab_reg_[0];
		for (int i=0; i<4; i++) {
			reading.digital_outputs[i] = x&1;
			x>>=1; 
		}
		//modbus_read_registers(mb_, 385, 1, tab_reg_);
		do385 = x = tab_reg_[1];
		for (int i=0; i<4; i++) {
			reading.digital_outputs[i+4] = x&1;
			x>>=1; 
		}

		// Read analog inputs 
		if(analog_inputs_ > 0){
			modbus_read_registers(mb_, 192, 2, tab_reg_);
			reading.analog_inputs[0] = double (tab_reg_[0] / 30000.0) * 10.0;  // Range 0-10V
			reading.analog_inputs[1] = double (tab_reg_[1] / 30000.0) * 10.0;  // Range 0-10V
		}
		// Read analog outputs
		if(analog_outputs_ > 0){
			modbus_read_registers(mb_, 578, 2, tab_reg_);
			reading.analog_outputs[0] = double (tab_reg_[0] / 30000.0) * 10.0;  // Range 0-10V
			reading.analog_outputs[1] = double (tab_reg_[1] / 30000.0) * 10.0;  // Range 0-10V
		}
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
	bool write_digital_output(robotnik_msgs::set_digital_output::Request &req,
		     robotnik_msgs::set_digital_output::Response &res )
	{
		if ((req.output <= 0) || (req.output > this->digital_outputs_)) {
			res.ret = false;
			ROS_ERROR("modbus_io::write_digital_output: Error on the output number %d. Out of range [1 -> %d]", req.output, this->digital_outputs_);
			return false;
		}
		req.output -= 1;	// Internally the device uses outputs from [0 to max_outputs - 1]
		int iret;
		uint16_t reg_val, bit;
		if (req.output < MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS) {// 384
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
		if (iret < 0) 
			res.ret = false; 
		else 
			res.ret = true;
		return res.ret;
	}

	// Service write analog output
	bool write_analog_output(robotnik_msgs::set_analog_output::Request &req,
	     robotnik_msgs::set_analog_output::Response &res )
	{
		if ((req.output < 0) || (req.output > this->analog_outputs_)) {
			ROS_ERROR("modbus_io::write_analog_output: Error on the output number %d. Out of range [1 -> %d]", req.output, this->analog_outputs_);
			res.ret = false;
			return false;
		}
		req.output -= 1;	// Internally the device uses outputs from [0 to max_outputs - 1]
		if (req.value < 0.0) req.value = 0.0;
		if (req.value > 10.0) req.value = 10.0;
		uint16_t value = (uint16_t) ((req.value / 10.0) * 30000.0);
		int iret = modbus_write_register(mb_,578+req.output, value);
		ROS_INFO("request: output=%d, value=%5.4f", (int) req.output, req.value );
		if (iret < 0) res.ret = false; 
		else res.ret = true;
		return res.ret;
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
