#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <iostream>
#include <roko_robot/sensors.h>
#include <roko_robot/navigation.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


// TODO: Consider removing global variables. Right now it is fairly simple to do -
// just move them inside SubscribeAndPublish class as a private variables.
float X=0, Y=0, omega=0;//объявление глобальных переменных для координат и угла курса


class SubscribeAndPublish
{
public:
    SubscribeAndPublish() // This is the constructor
    {
      nav_path = nav_msgs::Path();

      sensorSub = n_.subscribe("roko/sensors_data", 10, &SubscribeAndPublish::sensor_callback, this);
      navigationPub = n_.advertise<roko_robot::navigation>("roko/navigation_data", 10);
      displayPub1 = n_.advertise<nav_msgs::Path>("rviz/path_nav", 10);
      displayPub2 = n_.advertise<geometry_msgs::PoseStamped>("rviz/pose_nav", 10);
    }

    // This callback function for sensor data
    void sensor_callback(const roko_robot::sensors msg)
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

      // TODO: Stricly speaking you are ignoring sensor packets sorting routine.
      // You are assumung that if you receive a imu packet (for example),
      // calculated robot velocities would be equal to zero, so this would not
      // mess up your solution.
      // And it is true, kinda. But as a program grows larger - you will experience
      // problems with this approach and finding this error (warning, really)
      // would be much harder...
      float X1, Y1, r=0.1, l=2*M_PI*r, omega_spd, speed_abs, omega_gyro, b=0.5, time=0.1;//объявление переменных
      omega_spd=(-right_wh_rot_speed+left_wh_rot_speed)*l/(4*M_PI*b);//рассчет угловой скорости по курсу
      speed_abs=(right_wh_rot_speed+left_wh_rot_speed)*l/(4*M_PI);//рассчет изменения модуля скорости
      omega=omega+omega_spd*time;//интегрирование методом прямоугольников и получение угла курса
      
      X1=cos(omega)*speed_abs*time;//
      Y1=sin(omega)*speed_abs*time;//рассчет расстояния по х и у, которое преодолевает робот за период time со скоростью speed_abs
      X=X+X1;//
      Y=Y+Y1;// рассчет пройденного пути
      //counter++;
      display_navigation_solution(X, Y, omega);
      publish_navigation_solution(X, Y, omega);
    }

    void display_navigation_solution(float x, float y, float omega)
    {
        geometry_msgs::PoseStamped msg = geometry_msgs::PoseStamped();
        msg.header.frame_id = "map";
        msg.header.seq = counter;
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.orientation.z = sin(omega / 2.0);
        msg.pose.orientation.w = cos(omega / 2.0);
        displayPub2.publish(msg);

        nav_path.header = msg.header;
        nav_path.poses.push_back(msg);
        displayPub1.publish(nav_path);
    }

    void publish_navigation_solution(float x, float y, float omega)
    {
        roko_robot::navigation msg = roko_robot::navigation();
        msg.timestamp = ros::Time::now().toSec() / 1000.0;
        msg.X = x;
        msg.Y = y;
        msg.Omega = omega;
        navigationPub.publish(msg);
    }

private:
    // ROS services
    ros::NodeHandle n_;
    ros::Subscriber sensorSub;
    ros::Publisher navigationPub;
    ros::Publisher displayPub1;
    ros::Publisher displayPub2;
    ros::Publisher len;
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
