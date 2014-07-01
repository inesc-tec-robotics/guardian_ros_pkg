/*
 * Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include "sstream"
#include <iostream>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class GuardianKeyboardTeleopNode
{
    private:
        double walk_vel_;
        double run_vel_;
        double yaw_rate_;
        double yaw_rate_run_;
        
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        GuardianKeyboardTeleopNode()
        {
            pub_ = n_.advertise<geometry_msgs::Twist>("/guardian_controller/command", 1);
            
            //ROS_INFO("I am here vel!!!\n");
            ros::NodeHandle n_private("~");
            n_private.param("walk_vel", walk_vel_, 0.5);
            n_private.param("run_vel", run_vel_, 1.0);
            n_private.param("yaw_rate", yaw_rate_, 0.5);
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);
        }
        
        ~GuardianKeyboardTeleopNode() { }
        void keyboardLoop();
        
        void stopRobot()
        {
            cmdvel_.linear.x = 0.0;
            cmdvel_.angular.z = 0.0;
            pub_.publish(cmdvel_);
        }
};

GuardianKeyboardTeleopNode* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;


void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}


int main(int argc, char** argv)
{

    //ROS_INFO("I am here\n");
    ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    GuardianKeyboardTeleopNode tbk;
    
    signal(SIGINT, quit);

    boost::thread t = boost::thread(boost::bind(&GuardianKeyboardTeleopNode::keyboardLoop, &tbk));
    //ROS_INFO("I am here 2\n");
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();
    
    return(0);
}



void GuardianKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed = 0;
    int turn = 0;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
   puts("\r\rReading from keyboard");
   puts("\r\rUse WASD keys to control the robot");
   puts("\r\rPress Shift to move faster");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty == true)
            {
                stopRobot();
                dirty = false;
            }
            
            continue;
        }
        
        switch(c)
        {
            case KEYCODE_W:
                max_tv = walk_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
               //puts("straight\r");
                //std::cout<<"\rstraight";
                //printf("straight");
                break;
            case KEYCODE_S:
                max_tv = walk_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
               //puts("\r\rbackwards");
                break;
            case KEYCODE_A:
                max_rv = yaw_rate_;
                speed = 0;
                turn = 1;
                dirty = true;
               //puts("\r\rturn anti-clock");
                break;
            case KEYCODE_D:
                max_rv = yaw_rate_;
                speed = 0;
                turn = -1;
                dirty = true;
               //puts("\r\rturn clock");
                break;
                
            case KEYCODE_W_CAP:
                max_tv = run_vel_;
                speed = 1;
                turn = 0;
                dirty = true;
               //puts("\r\rstraight");
                break;
            case KEYCODE_S_CAP:
                max_tv = run_vel_;
                speed = -1;
                turn = 0;
                dirty = true;
               //puts("\r\rbackwards");
                break;
            case KEYCODE_A_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = 1;
                dirty = true;
               //puts("\r\rturn anti-clock");
                break;
            case KEYCODE_D_CAP:
                max_rv = yaw_rate_run_;
                speed = 0;
                turn = -1;
                dirty = true;
               //puts("\r\rturn anti-clock");
                break;
                
            default:
                max_tv = walk_vel_;
                max_rv = yaw_rate_;
                speed = 0;
                turn = 0;
                dirty = false;
        }
        
        cmdvel_.linear.x = speed * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}

