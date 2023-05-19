#!/usr/bin/env python3
import rospy
from math import sqrt, pi, copysign
from rdk_msgs.msg import control as cont
from rdk_msgs.msg import navigation as nav

from roko_robot.msg import control
from roko_robot.msg import navigation

class SubscribeAndPublish:

    def __init__(self):
        self._lastCoords = [0, 0, 0]
        self._lastTime = 0
        self._controls = [0, 0]

        self.navSub = rospy.Subscriber('navigation_data', nav, self.navigation_callback)
        self.ctrlSub = rospy.Subscriber('roko/control_data', control, self.control_callback)

        self.ctrlPub = rospy.Publisher('control_commands', cont, queue_size=3)
        self.navPub = rospy.Publisher('roko/navigation_data_true', navigation, queue_size=3)
        rospy.loginfo("[control_bridge] is ready!")
    # End of __init__()

    def pi2pi(self, angle):
        while abs(angle) > pi:
            angle -= copysign(2 * pi, angle)
        return angle

    def navigation_callback(self, msg):
        nmsg = navigation()
        nmsg.timestamp = int(rospy.get_time() * 1000)
        nmsg.X = msg.X
        nmsg.Y = msg.Y
        nmsg.Velocity = self._controls[0]
        nmsg.Rate = self._controls[1]
        nmsg.Omega = msg.heading
        #if self._lastTime != 0 and nmsg.timestamp != self._lastTime:
        #    nmsg.Velocity = sqrt((msg.X - self._lastCoords[0])**2 + (msg.Y - self._lastCoords[1])**2 ) / (nmsg.timestamp - self._lastTime) / 1000
        #    nmsg.Rate = self.pi2pi(msg.heading - self._lastCoords[2]) / (nmsg.timestamp - self._lastTime) / 1000
        self._lastCoords = [msg.X, msg.Y, msg.heading]
        self._lastTime = nmsg.timestamp
        self.navPub.publish(nmsg)
    # End of navigation_callback()

    def control_callback(self, msg):
        l = 0.1 * 2 * pi
        b = 0.5
        cmsg = cont()
        cmsg.ups = (msg.left + msg.right) * l / 4 / pi
        cmsg.dth = (msg.left - msg.right) * l / 4 / pi / b
        self._controls = [cmsg.ups, cmsg.dth]
        #cmsg.timestamp = round(rospy.get_time() * 1000) # assumed to be milliseconds
        cmsg.mode = 0 # automatic control
        self.ctrlPub.publish(cmsg)
    # End of control_callback()


def main():
    rospy.init_node('control_bridge')
    SAP = SubscribeAndPublish()

    rospy.spin()



if __name__ == "__main__":
    main()
