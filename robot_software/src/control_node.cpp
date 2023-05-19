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
    controlPub = nh.advertise<roko_robot::control>("roko/control_data", 10);
    ros::Duration(0.5).sleep(); // sleep for half a second

    roko_robot::control msg;
    msg.timestamp = ros::Time::now().toSec() * 1000;
    msg.left = 2.0;
    msg.right = 2.0;

    //controlPub.publish(msg);
    printf("Control command %5.2f %5.2f published\n", (float)msg.left, (float) msg.right);

    // If you want the program to just run once
    //ros::spinOnce();

    // For cycling operation use
    ros::Rate rate(1); // ROS Rate at 1 Hz
    int i = 0;
    bool isRotate = false;
    bool isSpeedUp = false;
    int nextRotate = 12;
    int nextSpeedUp = 14;
    int a;
  	while (ros::ok()) {
		    // do stuff
        if (isRotate)
        {
          if (isSpeedUp)
          {
            msg.left = 4.0;
            msg.right = 4.0;
          }
          else
          {
            msg.left = 4.0 * sqrt(2);
            msg.right = 4.0 * sqrt(2);
          }
          isSpeedUp = false;
          isRotate = false;
        }

        if (i == nextRotate)
        { 
          if ((i % 24) == 0)
          { 
            a = 2;
          }
          else
          {
            a = - 1;
          }
          msg.right = 7.0 * a;
          msg.left = -7.0 * a;
          isRotate = true;
          nextRotate = nextRotate + 12;
        }

        if (i == nextSpeedUp)
        {
          isSpeedUp = true;
          nextSpeedUp = nextSpeedUp + 28;
        }

        i++;

        controlPub.publish(msg);
        rate.sleep();
    }


  	return 0;
  }
