#include <ros/ros.h>
#include <robotnik_msgs/ptz.h>
#include <iostream>
#include <string>
#include <sensor_msgs/JointState.h>
#include <time.h>
#include <vector>

#define PI  3.14159265359

sensor_msgs::JointState joint_state;
double joint_pos = 0;

//Read laser position
void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_state = *msg;

  std::vector<std::string> joint_name = joint_state.name;
  int tilt_laser = std::find (joint_name.begin(),joint_name.end(), std::string("hokuyo_front_laser_tilt_joint")) - joint_name.begin();

  joint_pos = joint_state.position[tilt_laser]*180/PI; //laser joint position

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_tilt_node");
  ros::NodeHandle n;

  ROS_INFO("############### Laser tilt Command:");

  bool ptz_relative = false;
  std::string cmd_topic_ptz;
  double tilt_increment_degrees,laser_upper_limit_degrees,laser_lower_limit_degrees, epsilon, period;

  n.param<std::string>("cmd_topic_ptz", cmd_topic_ptz, "/guardian/command_ptz");
  n.param("ptz_relative", ptz_relative, true);
  n.param("tilt_increment_degrees", tilt_increment_degrees, 1.0);
  n.param("epsilon", epsilon, 0.5);
  n.param("period", period, 0.1);
  n.param("laser_upper_limit_degrees", laser_upper_limit_degrees, 10.0);
  n.param("laser_lower_limit_degrees", laser_lower_limit_degrees, -10.0);
  /********************************
   * Beware: Upper limit is closest to the floor and lower limit is closest to the ceiling - this is because of how the frame was defined
   * ******************************/


  std::cout<< "  ptz_relative: " << ptz_relative << std::endl;
  std::cout<< "  cmd_topic_ptz: " << cmd_topic_ptz << std::endl;
  std::cout<< "  tilt_increment: " << tilt_increment_degrees <<" degree(s)"<< std::endl;
  std::cout<< "  laser_upper_limit_degrees: " << laser_upper_limit_degrees  <<" degree(s)" << std::endl;
  std::cout<< "  laser_lower_limit_degrees: " << laser_lower_limit_degrees  <<" degree(s)" << std::endl;
  std::cout<< "  epsilon: " << epsilon << std::endl;
  std::cout<< "  period: " << period << std::endl;


  ros::Publisher pub_cmd = n.advertise<robotnik_msgs::ptz>(cmd_topic_ptz, 1); //Advertise tilt topic
  ros::Subscriber joint_state_sub = n.subscribe<sensor_msgs::JointState>("/guardian/joint_states", 1, jointStateCallback); //subscribes to joint_states to read laser position

  double num_increments = (laser_upper_limit_degrees - laser_lower_limit_degrees) / tilt_increment_degrees; //number of tilt increments
  double rate = num_increments / period; //period for each increment

  ros::Rate loop_rate(rate);

  robotnik_msgs::ptz ptz;

  ptz.pan = ptz.zoom = 0.0;
  ptz.tilt = tilt_increment_degrees;
  ptz.relative = ptz_relative;

  while (ros::ok() && ~ros::isShuttingDown() )
  {
	  ros::spinOnce();

	  //compare if laser reached limits
	  // because of rounding errors we bound the error using epsilon
	  double diff_up = laser_upper_limit_degrees - joint_pos;
	  double diff_low = joint_pos - laser_lower_limit_degrees;

	  if (diff_up <= epsilon) //reached limit closest to the floor
		  ptz.tilt = tilt_increment_degrees; // goes up

	  if (diff_low <= epsilon) //reached limit closest to the floor
		  ptz.tilt = -1*tilt_increment_degrees; // goes down

 	  pub_cmd.publish(ptz);
	  loop_rate.sleep();
  }

  return 0;
}
