#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <roko_robot/control.h>


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "control_node");
  	ros::NodeHandle nh;
    ros::Publisher controlPub;
    controlPub = nh.advertise<roko_robot::control>("control_data", 10);


    roko_robot::control msg;
    msg.timestamp = ros::Time::now().toSec() * 1000;
    msg.left = 8.5;
    msg.right = 8.3;

    controlPub.publish(msg);
    printf("Control command %5.2f %5.2f published\n", (float)msg.left, (float) msg.right);

    ros::spinOnce();

    // For cycling operation use
  	/*
    ros::Rate rate(2); // ROS Rate at 2Hz
  	while (ros::ok()) {
		    // do stuff
        rate.sleep();
    }
    */

  	return 0;
  }
