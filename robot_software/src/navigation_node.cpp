#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <roko_robot/sensors.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>



class SubscribeAndPublish
{
public:

    SubscribeAndPublish() // This is the constructor
    {
      nav_path = nav_msgs::Path();

      sensorSub = n_.subscribe("roko/sensors_data", 10, &SubscribeAndPublish::sensor_callback, this);
      displayPub1 = n_.advertise<nav_msgs::Path>("rviz/pose", 10);
      displayPub2 = n_.advertise<geometry_msgs::PoseStamped>("rviz/path", 10);
    }


    // This callback function continuously executes and reads the image data
    void sensor_callback(const roko_msgs::sensors msg)
    {
      // Decode the incoming sensor measurements
      float left_wh_rot_speed = msg.odom[0];
      float right_wh_rot_speed = msg.odom[1];
      float gyroX = msg.gyro[0];
      float gyroY = msg.gyro[1];
      float gyroZ = msg.gyro[2];
      float accX = msg.accl[0];
      float accY = msg.accl[1];
      float accZ = msg.accl[2];
      float gps1lat = msg.gps1_pos[0];  // deg
      float gps1lon = msg.gps1_pos[1];  // deg
      float gps1vn = msg.gps1_vel[0];  // m/s (north)
      float gps1ve = msg.gps1_vel[1];  // m/s (east)
      float gps2lat = msg.gps2_pos[0];  // deg
      float gps2lon = msg.gps2_pos[1];  // deg
      float gps2vn = msg.gps2_vel[0];  // m/s (north)
      float gps2ve = msg.gps2_vel[1];  // m/s (east)

      // TODO: calculate robot coordinates based on odometry control_data
      float X = counter * 0.1;
      float Y = counter * 0.2;


      counter++;
      display_navigation_solution(X, Y);
    }


    void display_navigation_solution(float x, float y)
    {
        geometry_msgs::PoseStamped msg = geometry_msgs::PoseStamped();
        msg.header.frame_id = "map";
        msg.header.seq = counter;
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        displayPub1.publish(msg);

        nav_path.header = msg.header;
        nav_path.poses
    }



private:
    // ROS services
    ros::NodeHandle n_;
    ros::Subscriber sensorSub;
    ros::Publisher displayPub1;
    ros::Publisher displayPub2;
    uint counter = 0;
    nav_msgs::Path nav_path;


};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_node");

    // Handle ROS communication events
    SubscribeAndPublish SAPObject;

    ros::spin();


  	return 0;
  }
