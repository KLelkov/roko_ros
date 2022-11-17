#!/usr/bin/env python3
import math
import numpy as np
import rospy

from roko_robot.msg import navigation
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
from matplotlib.animation import FuncAnimation


class SubscribeAndPublish:

    def __init__(self):

        self.screen = rospy.get_param("~screen", True)

        self._navSub = rospy.Subscriber('roko/navigation_data', navigation, self.navigation_callback)
        self._trueSub = rospy.Subscriber('rviz/pose', PoseStamped, self.true_callback)

        # ROS variables
        self._navSub = 0

        self.x_nav = []
        self.y_nav = []
        self.x_true = []
        self.y_true = []
        self.heading_nav = []
        self.heading_true = []
        self.pos_error = []
        self.time = []
        self.last_x = 0
        self.last_y = 0
        self.time0 = rospy.get_time()

        if self.screen:
            gridsize = (1,2)
            self.fig = plt.figure(figsize=(14,7))
            self.ax1 = plt.subplot2grid(gridsize,(0,0), rowspan=2)
            self.ax2 = plt.subplot2grid(gridsize,(0,1))
            self.ln_nav,  = self.ax1.plot([], [], 'r', linewidth=2, label='traj_nav')
            self.ln_true,  = self.ax1.plot([], [], 'g', linewidth=2, label='traj_true')
            self.ln_err, = self.ax2.plot([], [], 'b', linewidth=2, label='error')

            ani = FuncAnimation(self.fig, self.update_plot, init_func=self.plot_init)

            plt.show(block=True)

    def plot_init(self):
        self.ax1.set_title('Robot trajectory')
        self.ax1.set_xlim(-10, 10)
        self.ax1.set_ylim(-10, 10)
        self.ax1.grid()
        self.ax1.set_xlabel('Y (local), m')
        self.ax1.set_ylabel('X (local), m')
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.legend()

        self.ax2.set_title('Navigation error')
        self.ax2.grid()
        self.ax2.set_ylim(0, 10.0)
        self.ax2.set_xlim(0, 250)
        self.ax2.set_xlabel('Time, sec')
        self.ax2.set_ylabel('Navigation error, m')

        return self.ln_nav

    def update_plot(self, frame):
        self.ln_nav.set_data(self.x_nav, self.y_nav)
        self.ln_true.set_data(self.x_true, self.y_true)
        self.ln_err.set_data(self.time, self.pos_error)
        return (self.ln_nav, self.ln_true, self.ln_err)


    def navigation_callback(self, msg):
        self.y_nav.append(msg.X)
        self.x_nav.append(msg.Y)
        self.heading_nav.append(msg.Omega)
        self.time.append(rospy.get_time() - self.time0)
        err = math.sqrt((msg.X - self.last_x)**2 + (msg.Y - self.last_y)**2)
        self.pos_error.append(err)
        #print(self.pos_error)

    def true_callback(self, msg):
        self.y_true.append(msg.pose.position.x)
        self.x_true.append(msg.pose.position.y)
        #self.heading_nav.append(msg.heading)
        self.last_x = msg.pose.position.x
        self.last_y = msg.pose.position.y


def main():
    rospy.init_node('analyzer_node')
    SAP = SubscribeAndPublish()
    rospy.spin()


if __name__ =="__main__":
    main()
