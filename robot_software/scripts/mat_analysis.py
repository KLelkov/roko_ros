#!/usr/bin/env python3
import math
import numpy as np
import rospy
import csv
import os

from roko_robot.msg import navigation
from geometry_msgs.msg import PoseStamped
from roko_robot.msg import control

import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
from matplotlib.animation import FuncAnimation


class SubscribeAndPublish:

    def __init__(self):

        self.screen = rospy.get_param("~screen", True)
        script_dir = os.path.dirname(__file__)
        print("output directory: {}".format(script_dir))
        self.f = open(script_dir + '/log.csv', 'w')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['time', 'Xt', 'Yt', 'Ht', 'Xn', 'Yn', 'Hn', 'Error', 'Rate', 'Velocity', 'Control_l', 'Control_r'])

        self._trueSub = rospy.Subscriber('rviz/pose', PoseStamped, self.true_callback)
        self._navSub = rospy.Subscriber('roko/navigation_data', navigation, self.navigation_callback)
        self._ctrlSub = rospy.Subscriber('roko/control_data', control, self.control_callback)


        # ROS variables
        self._navSub = 0

        self.ctrl_left = [0]
        self.ctrl_right = [0]
        self.nav_vel = []
        self.nav_rate = []
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
        self.nav_vel.append(msg.Velocity)
        self.nav_rate.append(msg.Rate)
        #print(self.pos_error)
        #self.writer.writerow([self.time[-1], self.x_true[-1], self.y_true[-1], 0, self.x_nav[-1], self.y_nav[-1], self.heading_nav[-1], self.pos_error[-1]])

    def control_callback(self, msg):
        self.ctrl_left.append(msg.left)
        self.ctrl_right.append(msg.right)
      
        #self.writer.writerow([self.time[-1], self.x_true[-1], self.y_true[-1], 0, self.x_nav[-1], self.y_nav[-1], self.heading_nav[-1], self.pos_error[-1]])

    def true_callback(self, msg):
        self.y_true.append(msg.pose.position.x)
        self.x_true.append(msg.pose.position.y)
        self.heading_true.append(math.asin(msg.pose.orientation.z) * 2)
        self.last_x = msg.pose.position.x
        self.last_y = msg.pose.position.y

    def save_to_file(self):

        if len(self.time) > 10:
            print(f"Saving data at time {self.time[-1]}")
            self.writer.writerow([self.time[-1], self.x_true[-1], self.y_true[-1], self.heading_true[-1], self.x_nav[-1], self.y_nav[-1], self.heading_nav[-1], self.pos_error[-1],
                                  self.nav_rate[-1], self.nav_vel[-1], self.ctrl_left[-1], self.ctrl_right[-1]])


def main():
    rospy.init_node('analyzer_node')
    SAP = SubscribeAndPublish()
    #rospy.spin()
    r = rospy.Rate(10) # 100 Hz
    while not rospy.is_shutdown():
        SAP.save_to_file()
        d = 0
        r.sleep()
    SAP.f.close()


if __name__ =="__main__":
    main()
