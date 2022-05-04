#!/usr/bin/env python3

import rospy
import math
import numpy as np
from roko_robot.msg import control
from roko_robot.msg import navigation


class SubscribeAndPublish:

    def __init__(self):

        # Create publisher for publishing sensor measurements
        self._navigationSub = rospy.Subscriber('roko/navigation_data', navigation, self.navigation_callback)
        self._controlPub = rospy.Publisher('roko/control_data', control, queue_size=10)

    # This function will be called each time a new navigation message is received.
    # So basicly - every time you calculated your navigation solution in navigation_node
    def navigation_callback(self, msg):
        # Extract navigation values from the message
        timestamp = msg.timestamp
        x = msg.X
        y = msg.Y
        omega = msg.Omega

        # TODO: Integrate your old control algo here

        # TODO: Adjust the control algo to take into account current navigation
        # parameters. AKA negative feedback loop.
        left_speed = 2.0
        right_speed = 2.0

        # Send control values to the robot
        self.publishControl(left_speed, right_speed)


    def publishControl(self, left_speed, right_speed):
        msg = control()
        msg.timestamp = int(rospy.get_time() * 1000)
        msg.left = left_speed  # rotation speed of the left wheel
        msg.right = right_speed  # rotation speed of the right wheel
        self._controlPub.publish(msg)
        print("Control command {:.2f} {:.2f} published".format(left_speed, right_speed))

# End of SubscribeAndPublish class


def main():
    rospy.init_node('py_control_node')
    SAP = SubscribeAndPublish()

    rospy.spin()
# End on main()


if __name__ =="__main__":
    main()
