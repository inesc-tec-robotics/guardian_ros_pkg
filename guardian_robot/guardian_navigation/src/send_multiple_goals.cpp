/************************************************************************************************
* Adapted from SendingSimpleGoals: http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals *
*
* Sends a new goal, from an array of goals, when the previous is achieved or fails for some reason
* Goals are in map frame
*************************************************************************************************/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "multiple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_WARN("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  int n_goals = 3;
  geometry_msgs::Pose goals[n_goals];


//GUARDIAN GOALS // goal pose must be in map frame
  goals[0].position.x =  -6.0;
  goals[0].position.y =  -4.0;
  goals[0].position.z =  0.0;
  goals[0].orientation.x =  0.0;
  goals[0].orientation.y =  0.0;
  goals[0].orientation.z =  1.0;
  goals[0].orientation.w =  0.0;

  goals[1].position.x =  -8.5;
  goals[1].position.y =  -2.5;
  goals[1].position.z =  0.0;
  goals[1].orientation.x =  0.0;
  goals[1].orientation.y =  0.0;
  goals[1].orientation.z =  0.71;
  goals[1].orientation.w =  0.71;

  goals[2].position.x =  -9.0;
  goals[2].position.y =  -2.0;
  goals[2].position.z =  0.0;
  goals[2].orientation.x =  0.0;
  goals[2].orientation.y =  0.0;
  goals[2].orientation.z =  1.0;
  goals[2].orientation.w =  0.0;



  for (int i=0; i < n_goals; ++i)
  {

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = goals[i].position.x;
      goal.target_pose.pose.position.y = goals[i].position.y;
      goal.target_pose.pose.position.z = goals[i].position.z;
      goal.target_pose.pose.orientation.z = goals[i].orientation.z;
      goal.target_pose.pose.orientation.w =goals[i].orientation.w;

      std::cout << BLUE << ">>> " << RESET <<"Sending goal #" << i+1 << std::endl;
      ac.sendGoal(goal); // push the goal to move_base node for processing

      // wait for the goal to finish.
      ac.waitForResult();  // blocks until the move_base action is done processing the goal we sent i

      // check for success/failure and inform about it
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        std::cout << GREEN << "GOAL # " << i+1 << " READCHED " << RESET << std::endl;
      else
        std::cout << RED << "FAILED TO REACH GOAL # " << i+1 << RESET << std::endl;

  }

  //std::cout << BLUE << "************************" << RESET << std::endl;
  return 0;
}
