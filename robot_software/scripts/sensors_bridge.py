#!/usr/bin/env python3
import rospy
from rdk_msgs.msg import locomotion
from rdk_msgs.msg import motors
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped

from roko_robot.msg import sensors

class SubscribeAndPublish:

    def __init__(self):
        self._gpsFlags = [0, 0, 0, 0]
        self._gpsData = [[0, 0], [0, 0], [0, 0], [0, 0]]

        self._odoSub = rospy.Subscriber('motors_data', motors, self.odo_callback)
        self._imuSub = rospy.Subscriber('locomotion_data', locomotion, self.imu_callback)
        self._posSub = rospy.Subscriber('gps_node/fix', NavSatFix, self.pos_callback)
        self._velSub = rospy.Subscriber('gps_node/fix_velocity', TwistWithCovarianceStamped, self.vel_callback)
        self._pos2Sub = rospy.Subscriber('gps_node2/fix', NavSatFix, self.pos2_callback)
        self._vel2Sub = rospy.Subscriber('gps_node2/fix_velocity', TwistWithCovarianceStamped, self.vel2_callback)

        self._sensorPub = rospy.Publisher('roko/sensors_data', sensors, queue_size=5)
        rospy.loginfo("[sensor_bridge] is ready!")
    # End of __init__()


    def odo_callback(self, msg):
        odo_left = 0
        if abs(msg.odo[0] > 0.2):
            odo_left = msg.odo[0]
        odo_right = 0
        if abs(msg.odo[1] > 0.2):
            odo_right = - msg.odo[1]
        
        omsg = sensors()
        omsg.timestamp = int(rospy.get_time() * 1000)
        omsg.odom[0] = odo_left # rotation speed of the left wheel
        omsg.odom[1] = odo_right # rotation speed of the right wheel

        self._sensorPub.publish(omsg)
    # End of odo_callback()

    def imu_callback(self, msg):

        omsg = sensors()
        omsg.timestamp = int(rospy.get_time() * 1000)
        # Keep in mind that IMU readings are in BODY frame
        omsg.gyro[0] = 0.0  # rotation speed on X axis
        omsg.gyro[1] = 0.0  # rotation speed on Y axis
        omsg.gyro[2] = -msg.angular_velocity[2]  # rotation speed on Z axis
        omsg.accl[0] = 0.0  # acceleration on X axis
        omsg.accl[1] = 0.0  # acceleration on Y axis
        omsg.accl[2] = 0.0  # acceleration on Z axis

        self._sensorPub.publish(omsg)
    # End of imu_callback()

    def pos_callback(self, msg):
        newLat = msg.latitude
        newLon = msg.longitude
        if newLat != 0 and newLon != 0:
            self._gpsData[0][0] = newLat
            self._gpsData[0][1] = newLon
            self._gpsFlags[0] = 1
    # End of pos_callback()

    def pos2_callback(self, msg):
        newLat = msg.latitude
        newLon = msg.longitude
        if newLat != 0 and newLon != 0:
            self._gpsData[2][0] = newLat
            self._gpsData[2][1] = newLon
            self._gpsFlags[2] = 1
    # End of pos2_callback()

    def vel_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        if abs(vx) < 10 and abs(vy) < 10:
            self._gpsData[1][0] = vx
            self._gpsData[1][1] = vy
            self._gpsFlags[1] = 1
    # End of vel_callback()

    def vel2_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        if abs(vx) < 10 and abs(vy) < 10:
            self._gpsData[3][0] = vx
            self._gpsData[3][1] = vy
            self._gpsFlags[3] = 1
    # End of vel2_callback()

    def check_gps_data(self):
        if sum(self._gpsFlags) == 4:
            msg = sensors()
            msg.timestamp = int(rospy.get_time() * 1000)
            # Keep in mind that GPS readings are in GEO frame
            msg.gps1_pos[0] = self._gpsData[0][0] # Latitude
            msg.gps1_pos[1] = self._gpsData[0][1] # Longitude
            msg.gps1_vel[0] = self._gpsData[1][0] # velocity on X (north)
            msg.gps1_vel[1] = self._gpsData[1][1] # velocity on Y (east)

            msg.gps2_pos[0] = self._gpsData[2][0] # Latitude
            msg.gps2_pos[1] = self._gpsData[2][1] # Longitude
            msg.gps2_vel[0] = self._gpsData[3][0] # velocity on X (north)
            msg.gps2_vel[1] = self._gpsData[3][1] # velocity on Y (east)

            self._sensorPub.publish(msg)
            self._gpsFlags = [0, 0, 0, 0]
    # End of imu_callback()


def main():
    rospy.init_node('sensors_bridge')
    SAP = SubscribeAndPublish()

    rospy.spin()



if __name__ == "__main__":
    main()
