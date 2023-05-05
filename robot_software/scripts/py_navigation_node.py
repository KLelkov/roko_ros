#!/usr/bin/env python3

import rospy
import math
import numpy as np
from roko_robot.msg import navigation
from roko_robot.msg import sensors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class SubscribeAndPublish:

    def __init__(self):
        self.nav_path = Path()
        self.counter = 0

        # Create publisher for publishing sensor measurements
        self._sensorsSub = rospy.Subscriber('roko/sensors_data', sensors, self.sensor_callback)
        self._navigationPub = rospy.Publisher('roko/navigation_data', navigation, queue_size=2)
        self._displayPub1 = rospy.Publisher('rviz/path_nav', Path, queue_size=2)
        self._displayPub2 = rospy.Publisher('rviz/pose_nav', PoseStamped, queue_size=2)

    # This callback function for sensor data
    def sensor_callback(self, msg):
        # Decode the incoming sensor measurements
        left_wh_rot_speed = msg.odom[0]
        right_wh_rot_speed = msg.odom[1]
        gyroX = msg.gyro[0]
        gyroY = msg.gyro[1]
        gyroZ = msg.gyro[2]
        accX = msg.accl[0]
        accY = msg.accl[1]
        accZ = msg.accl[2]
        gps1lat = msg.gps1_pos[0]  # deg
        gps1lon = msg.gps1_pos[1]  # deg
        gps1vn = msg.gps1_vel[0]  # m/s (north)
        gps1ve = msg.gps1_vel[1]  # m/s (east)
        gps2lat = msg.gps2_pos[0]  # deg
        gps2lon = msg.gps2_pos[1]  # deg
        gps2vn = msg.gps2_vel[0]  # m/s (north)
        gps2ve = msg.gps2_vel[1]  # m/s (east)


        # TODO: Integrate your old navigation algo here
        X = 0
        Y = 0
        omega = 0
        vel = 0
        rate = 0

        self.display_navigation_solution(X, Y, omega);
        self.publish_navigation_solution(X, Y, omega, vel, rate);

    def display_navigation_solution(self, x, y, omega):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.seq = self.counter
        self.counter += 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(omega / 2.0)
        msg.pose.orientation.w = math.cos(omega / 2.0)
        self._displayPub2.publish(msg)

        self.nav_path.header = msg.header
        self.nav_path.poses.append(msg)
        self._displayPub1.publish(self.nav_path)


    def publish_navigation_solution(self, x, y, omega, velocity, rate):
        msg = navigation()
        msg.timestamp = int(rospy.get_time() * 1000)
        msg.X = x
        msg.Y = y
        msg.Velocity = velocity
        msg.Rate = rate
        msg.Omega = omega
        self._navigationPub.publish(msg)

# End of SubscribeAndPublish class


def main():
    rospy.init_node('py_navigation_node')
    SAP = SubscribeAndPublish()

    rospy.spin()
# End on main()


if __name__ =="__main__":
    main()
