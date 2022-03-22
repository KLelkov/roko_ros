#!/usr/bin/env python3

import rospy
import math
import numpy as np
import Roko
from std_msgs.msg import String
from roko_robot.msg import sensors


class SubscribeAndPublish:

    def __init__(self):
        # Set initial values
        self._robot = Roko.Roko(0, 0, 0, 0, 0)

        # Create publisher for publishing sensor measurements
        self._sensorPub = rospy.Publisher('sensors_data', sensors, queue_size=10)
        # Create publisher for publishing true robot motion parameters
        self._truePub = rospy.Publisher('true_parameters', String, queue_size=10)
        self._controlSub = rospy.Subscriber('control_data', String, self.control_callback)


    # Used to limit angle within [-pi, pi] limits
    def pi2pi(self, angle, metric='rad'):
        edgeValue = math.pi # rad
        if metric == 'deg':
            edgeValue = 180.0 # deg
        while angle <= -edgeValue or angle > edgeValue:
            angle = angle - math.copysign(2 * edgeValue, angle)
        return angle


    def control_callback(self, msg):
        # Extract control values from the message
        left = msg.odo[0]
        right = msg.odo[1]

        # Pass these values to the robot
        self._robot.set_motion(left, right)


    def publishOdometry(self):
        # Gather odometry measurements from the robot
        odo_readings = self._robot.get_odometry()
        msg = sensors()
        msg.timestamp = int(rospy.get_time() * 1000)
        msg.odom[0] = odo_readings[0] # rotation speed of the left wheel
        msg.odom[1] = odo_readings[1] # rotation speed of the right wheel

        self._sensorPub.publish(msg)


    def publishIMU(self):
        # Gather IMU measurements from the robot
        imu_readings = self._robot.get_imu()
        msg = sensors()
        msg.timestamp = int(rospy.get_time() * 1000)
        # Keep in mind that IMU readings are in BODY frame
        msg.gyro[0] = imu_readings[0] # rotation speed on X axis
        msg.gyro[1] = imu_readings[1] # rotation speed on Y axis
        msg.gyro[2] = imu_readings[2] # rotation speed on Z axis
        msg.accl[0] = imu_readings[3] # acceleration on X axis
        msg.accl[1] = imu_readings[4] # acceleration on Y axis
        msg.accl[2] = imu_readings[5] # acceleration on Z axis

        self._sensorPub.publish(msg)


    def publishGPS(self):
        # Gather GPS measurements from the robot
        gps_readings = self._robot.get_gps()
        msg = sensors()
        msg.timestamp = int(rospy.get_time() * 1000)
        # Keep in mind that GPS readings are in GEO frame
        msg.gps1_pos[0] = gps_readings[0] # X position
        msg.gps1_pos[1] = gps_readings[1] # Y position
        msg.gps1_vel[0] = gps_readings[2] # velocity on X (north)
        msg.gps1_vel[1] = gps_readings[3] # velocity on Y (east)

        msg.gps2_pos[0] = gps_readings[4] # X position
        msg.gps2_pos[1] = gps_readings[5] # Y position
        msg.gps2_vel[0] = gps_readings[6] # velocity on X (north)
        msg.gps2_vel[1] = gps_readings[7] # velocity on Y (east)

        self._sensorPub.publish(msg)


    # NOTE: variables with underscore (_[var]) are considered protected class variables
    # and should not be changed anywhere outside of this class.

    # Initial robot state
    # (X, Y, Psi, Velocity, Angular_rate)
    _robot = Roko.Roko(0, 0, 0, 0, 0)

    # ROS communication handlers
    _truePub = 0
    _sensorPub = 0
    _controlSub = 0

# End of SubscribeAndPublish class


def main():
    rospy.init_node('roko_node', anonymous=True)
    SAP = SubscribeAndPublish()
    r = rospy.Rate(100) # 100 Hz
    counter = 0
    while not rospy.is_shutdown():
        # Main cycle (executed every 0.01 seconds)
        counter = (counter + 1) % 1000

        SAP._robot.update()
        SAP.publishIMU()
        if counter % 10 == 0:
            SAP.publishOdometry()
        if counter % 100 == 0:
            SAP.publishGPS()

        r.sleep()
# End on main()


if __name__ =="__main__":
    main()
