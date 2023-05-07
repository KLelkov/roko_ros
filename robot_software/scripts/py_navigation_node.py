#!/usr/bin/env python3

import rospy
import math
import numpy as np
from roko_robot.msg import navigation
from roko_robot.msg import sensors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

dt=0.01 
P_hat=np.array([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 3, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
F_mat=np.array([[1, 0, 0, dt, 0, 0], [0, 1, 0, 0, dt, 0], [0, 0, 1, 0, 0, dt]])
pi=math.pi
S_mat=np.zeros((6, 6))
K_mat=np.zeros((6, 6))
X_hat=np.zeros((6, 6))
W=np.eye((6))
psi=0
psi_dot=0
I=np.eye(6)

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


        
        
        r=0.1
        l=2*pi*r
        b=0.5
        d=0.5
        B=np.array([[0, 0], [0, 0], [0, 0], [pi/(l*math.cos(psi)), pi/(l*math.cos(psi))], [0, 0]])
        U=np.array([right_wh_rot_speed, left_wh_rot_speed])

        def kalman_filter(x, p, z, f, h, b, u, w, v, i):
            # Предсказание
            q = np.diag(w.A1)  # матрица шумов системы
            x = np.matmul(f, x) + np.matmul(b, u) + w
            p = np.matmul(np.matmul(f, p), f.T) + q
            # Измерение
            r = np.diag(v.A1)  # матрица шумов измерений
            y = z - np.matmul(h, x)
            s = np.matmul(np.matmul(h, p), h.T) + r
            k = np.matmul(np.matmul(p, h.T), np.linalg.inv(s))
            x = x + np.matmul(k, y)
            p = np.matmul(i - np.matmul(k, h), p)

            return x, p

            
        #тут инициализация переменных для гнсс
        if gps1lat != 0:
            E1 = gps1lon * math.cos (gps1lat)* 111100
            E2 = gps2lon * math.cos (gps2lat)* 111100

            N1 = gps1lat * 111100
            N2 = gps2lat * 111100
            psi_gnss=math.atan2((E2-E1)/(N2-N1))
            
            E1_dot = gps1ve
            E2_dot = gps2ve - d * psi_dot * math.cos (psi)
            N1_dot = gps1vn
            N2_dot = gps2vn - d * psi_dot * math.sin (psi)
            psi_gnss_dot=math.atan2((E2-E1)/(N2-N1))
            H_GNSS=np.array([[0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
            Z_GNSS=np.array([E1, E2, N1, N2, psi_gnss, E1_dot, E2_dot, N1_dot, N2_dot, psi_gnss_dot])
        
        Z_odom=np.array([left_wh_rot_speed, right_wh_rot_speed])
        H_odom=np.array([[0, 0, 0, pi/(l*math.cos(psi)), pi/(l*math.cos(psi)), b*2*pi/l], [0, 0, 0, pi/(l*math.cos(psi)), pi/(l*math.cos(psi)), b*2*pi/l]])
        V_odom=np.array([0.01, 0.01])
        
        Z_INS=np.array([gyroZ, accX, accY])
        H_INS=np.array([[0, 0, 0, 0, 0, 1], [0, 0, 0, dt*math.sin(psi_dot), 0, 0], [0, 0, 0, 0, dt*math.cos(psi), 0]])
        V_INS=np.array([0.25, 0.25, 0.66])

        
        
        if gps1lat != 0:
             H=H_GNSS
             Z=Z_GNSS
             V=np.zeros(8)
        else:
            if right_wh_rot_speed != 0:
                H=H_odom
                Z=Z_odom
                V=V_odom
            else:
                H=H_INS
                Z=Z_INS
                V=V_INS
        global X_hat, P_hat, F_mat, I
        [X_hat, P_hat]=kalman_filter(X_hat, Z, P_hat,F_mat, H, B, U, W, V, I)


        X = P_hat[1,1]
        Y = P_hat[2,2]
        omega = P_hat[3,3]

        vel = sqrt(X**2+Y**2)
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
