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
#include "functions.h"





class SubscribeAndPublish
{
    float X = 0;
    float Y = 0;
    float omega = 0; //объявление глобальных переменных для координат и угла курса
    float lwspd = 0;
    float rwspd = 0;
//  счетчик проходов
    int j=0;
//  данные для гироскопа
    float legacy_gyro[5] = {0}; // last 5 values of gyro measurements
    float gyro_ZS=0;
//  данные для пары каселерометров
    float spd_X=0, spd_Y=0, spd_Z=0;
    float legacy_accX[5] = {0},legacy_accY[5] = {0};
    float accX_ZS=0, accY_ZS=0;

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

        //выявление смещения 0 ДЛЯ АКСЕЛЕРОМЕТРОВ И ГИРОСКОПОВ
      if (j<=19)
        {
          if (j==19) //условие какое по счету прохождение скрипта
                       {
                         gyro_ZS+=gyroZ; //прибавление нового измерения гироскопа
                         accX_ZS+=accX;
                         accY_ZS+=accY;
                         j++; 
                         gyro_ZS=gyro_ZS/j; //деление всей суммы на 20
                         accX_ZS=accX_ZS/j;
                         accY_ZS=accY_ZS/j;
                       }
                   else
                       {
                         gyro_ZS+=gyroZ;
                         accX_ZS+=accX;
                         accY_ZS+=accY;
                         j++;
                       }
                    }   

      // TODO: Gyroscope measurements actually might contain a permanent bias value.
      // You should calculate it while the robot is moving forward and average first, lets say, 20 measurements
      // Then you want to substract it from all further measurements.
      float omega_gyro=0, Xacc=0, Yacc=0, *p_legacy_gyro, *p_legacy_accX, *p_legacy_accY;
      p_legacy_gyro=&legacy_gyro[0];
      p_legacy_accX=&legacy_accX[0];
      p_legacy_accY=&legacy_accY[0];
      //фильтр 5 последних значений ДЛЯ АКСЕЛЛЕРОМЕТРОВ И ГИРОСКОПОВ
      for (int i = 0; i < 5; ++i) /* NOTE: 
      Consider incresing the number of filtering points More points will introduce the time lag to the system, 
      but if your robot is already biased to the right, that might not be such a bad thing
      */
      {
        if (i != 4)
        {
                legacy_gyro[i] = legacy_gyro[i+1];
                legacy_accX[i] = legacy_accX[i+1];
                legacy_accY[i] = legacy_accY[i+1];
        }
        else
        {
            if (j==20)
            {
                legacy_gyro[i] = gyroZ+gyro_ZS;
                legacy_accX[i] = accX+accX_ZS;
                legacy_accY[i] = accY+accY_ZS;
                omega_gyro= midle_value (p_legacy_gyro,sizeof(p_legacy_gyro)/sizeof(p_legacy_gyro[0]));
                Xacc=midle_value(p_legacy_accX,(sizeof(p_legacy_accX)/sizeof(p_legacy_accX[0])));
                Yacc=midle_value(p_legacy_accX,sizeof(p_legacy_accY)/sizeof(p_legacy_accY[0]));
                printf("Vx:%f\n",(Xacc));
                }
            else
            {
                legacy_gyro[i] = gyroZ;
                legacy_accX[i] = accX;
                legacy_accY[i] = accY;
                omega_gyro= midle_value (p_legacy_gyro,sizeof(p_legacy_gyro)/sizeof(p_legacy_gyro[0]));
                Xacc=midle_value(p_legacy_accX,(sizeof(p_legacy_accX)/sizeof(p_legacy_accX[0])));
                Yacc=midle_value(p_legacy_accX,sizeof(p_legacy_accY)/sizeof(p_legacy_accY[0]));
                printf("Vx:%f\n",(Xacc));
            }
         }
      }

      float X1, Y1, r=0.1, l=2*M_PI*r, omega_spd, speed_abs,  Wz, b=0.5, time=0.01, k=0.9;//объявление переменных
      
      if ((left_wh_rot_speed == 0) && (right_wh_rot_speed == 0))// If the sensor data doesn't contain odometry values - use the last ones
          {
               left_wh_rot_speed = lwspd;
               right_wh_rot_speed = rwspd;
               // NOTE: At this point speed_abs = 0, and you still use it to calculate X and Y
               // This means that nine times out of ten the robot thinks, that it is rotating in place (without moving forward)
          }
          else  
          {
                lwspd=left_wh_rot_speed;
                rwspd=right_wh_rot_speed;
          }   
      omega_spd=(-right_wh_rot_speed+left_wh_rot_speed)*l/(4*M_PI*b);//рассчет угловой скорости по курсу
      Wz=(k*omega_spd*time)+((1-k)*(omega_gyro)*time);//фильтр комплиментарный
      omega+=Wz;//интегрирование методом прямоугольников и получение угла курса
      float corX=coriolis(spd_X,omega_spd),corY=coriolis(spd_Y,omega_spd);
      printf("corX:%f \t corY:%f\n", corX, corY);
      spd_X=integ(spd_X,time,(Xacc));
      spd_Y=integ((Yacc-(coriolis(spd_Y,omega_spd))),time,spd_Y);
      
      speed_abs=(right_wh_rot_speed+left_wh_rot_speed)*l/(4*M_PI);//рассчет изменения модуля скорости
      X1=cos(omega)*speed_abs*time;//
      Y1=sin(omega)*speed_abs*time;//рассчет расстояния по х и у, которое преодолевает робот за период time со скоростью speed_abs
      X=X+X1;//рассчет пройденного
      Y=Y+Y1;// пути
      //printf(" Vx:%f Vy%f Vacc-Vodom:%f \n ",spd_X, spd_Y, (sqrt(pow(spd_X,2)+pow(spd_Y,2))-speed_abs));
      display_navigation_solution(X, Y, omega);
      publish_navigation_solution(X, Y, omega);
      printf("Vx:%f\n",(spd_X));
      printf("_______________________________\n");
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
