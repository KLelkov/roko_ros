#!/usr/bin/env python3

import rospy
import math
import numpy as np
from roko_robot.msg import navigation
from roko_robot.msg import sensors
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

dt=0.01 
P_hat=np.array([[100, 0, 0, 0, 0, 0], [0, 100, 0, 0, 0, 0], [0, 0, 100, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
F_mat=np.array([[1, 0, 0, dt, 0, 0], [0, 1, 0, 0, dt, 0], [0, 0, 1, 0, 0, dt], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
pi=math.pi
X_hat=np.array([[0], [0], [0], [0], [0], [0]])
W=np.array([[1e-7], [1e-7], [1e-7], [1e-8], [1e-8], [1e-5]])
psi=0
psi_dot=0
I=np.eye(6)
gps_zero1 = [0, 0]
gps_zero2 = [0, 0]
last_time = 0

class SubscribeAndPublish:

    def __init__(self):
        self.nav_path = Path()
        self.counter = 0

        # Create publisher for publishing sensor measurements
        self._sensorsSub = rospy.Subscriber('roko/sensors_data', sensors, self.sensor_callback, queue_size=1)
        self._navigationPub = rospy.Publisher('roko/navigation_data', navigation, queue_size=2)
        self._displayPub1 = rospy.Publisher('rviz/path_nav', Path, queue_size=2)
        self._displayPub2 = rospy.Publisher('rviz/pose_nav', PoseStamped, queue_size=2)

    def kalman_filter_predict(x, p, f, w, u):
        # Предсказание
        q = np.diag(w)  # матрица шумов системы
        #print(H)
        x = np.matmul(f, x) + u + w
        print(f"x: {x}")
        p = np.matmul(np.matmul(f, p), f.T) + q
        #print(f"Predict timespamp: {int(rospy.get_time() * 1000)}")
        return x, p
    
    def kalman_filter_update(x, p, z, h, v, i):
        # Измерение
        r = np.diag(v)  # матрица шумов измерений
        #print()
        y = z - np.matmul(h, x)
        s = np.matmul(np.matmul(h, p), h.T) + r
        k = np.matmul(np.matmul(p, h.T), np.linalg.inv(s))
        #print(k)
        #print(y)
        x = x + np.matmul(k, y)
        p = np.matmul(i - np.matmul(k, h), p)

        #print(f"Update timespamp: {int(rospy.get_time() * 1000)}")
        return x, p

    # This callback function for sensor data
    def sensor_callback(self, msg):
        global X_hat, P_hat, F_mat, I, psi, gps_zero1, gps_zero2, last_time
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
        timestamp = int(rospy.get_time() * 1000)  #msg.timestamp # milliseconds
        #timestamp = msg.timestamp # milliseconds


        if last_time != 0:
            dt = (timestamp - last_time) / 1000
            F_mat=np.array([[1, 0, 0, dt, 0, 0], [0, 1, 0, 0, dt, 0], [0, 0, 1, 0, 0, dt], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
            #print(f"time diff: {int(rospy.get_time() * 1000) - msg.timestamp} ms")
            #print(f"diff: {timestamp - last_time} dt: {dt}")
        
        #B=np.array([[0, 0], [0, 0], [0, 0], [pi/(l*math.cos(psi)), pi/(l*math.cos(psi))], [pi/(l*math.cos(psi)), pi/(l*math.cos(psi))], [b*pi/l, b*pi/l]])
        #U=np.array([[right_wh_rot_speed], [left_wh_rot_speed]])

        # IMU (100 Hz)
        if gps1lat == 0 and left_wh_rot_speed == 0 and right_wh_rot_speed == 0:
            # last_time = timestamp
            # # predict
            # [X_hat, P_hat] = SubscribeAndPublish.kalman_filter_predict(X_hat, P_hat, F_mat, W)
            # update
            Z_INS=np.array([[gyroZ]])
            H_INS=np.array([[0, 0, 0, 0, 0, 1]])
            V_INS=np.array([[7e-5]])  # 7e-5 = СКО дрейфа гироскопа
            [X_hat, P_hat] = SubscribeAndPublish.kalman_filter_update(X_hat, P_hat, Z_INS, H_INS, V_INS, I)

        # ODO (10 Hz)
        # условие
        # update

        if left_wh_rot_speed != 0 or right_wh_rot_speed != 0:
            r=0.1
            l=2*pi*r
            b=0.5
            last_time = timestamp
            velocity = (left_wh_rot_speed + right_wh_rot_speed) * l / 4 / pi
            rate = (left_wh_rot_speed - right_wh_rot_speed) * l / 4 / pi / b
            # Z_odom=np.array([[ velocity * math.cos(X_hat[2][0])], [velocity * math.sin(X_hat[2][0])], [rate]])
            # H_odom=np.array([[0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
            # V_odom=np.array([5e-1, 5e-1, 5e-2])
            #[X_hat, P_hat] = SubscribeAndPublish.kalman_filter_update(X_hat, P_hat, Z_odom, H_odom, V_odom, I)            

            u = np.array([[0], [0], [0], [velocity * math.cos(X_hat[2][0])], [velocity * math.sin(X_hat[2][0])], [rate]])
            # predict
            [X_hat, P_hat] = SubscribeAndPublish.kalman_filter_predict(X_hat, P_hat, F_mat, W, u)
            # Z_odom=np.array([[left_wh_rot_speed], [right_wh_rot_speed]])
            # H_odom=np.array([[0, 0, 0, 0, (2*pi)/(l*math.sin(X_hat[2][0])), b*2*pi/l], [0, 0, 0, (2*pi)/(l*math.cos(X_hat[2][0])), 0, -b*2*pi/l]])
            # V_odom=np.array([5e-2, 5e-2])

        # GPS (5 Hz)
        # условие
        # update
        if gps1lat!= 0 or gps2lon != 0:
            # rospy.logerr("--- GPS ---")
            d=0.5
            if gps_zero1[0] == 0:
                gps_zero1 = [gps1lat, gps1lon]
            if gps_zero2[0] == 0:
                gps_zero2 = [gps2lat, gps2lon]

            E1 = (gps1lon - gps_zero1[1]) * math.cos (gps1lat * pi / 180)* 111100
            E2 = (gps2lon - gps_zero2[1]) * math.cos (gps2lat * pi / 180)* 111100 - d * math.sin(X_hat[2][0])
            N1 = (gps1lat - gps_zero1[0]) * 111100
            N2 = (gps2lat - gps_zero2[0]) * 111100 - d * math.cos(X_hat[2][0])

            
            
            E1_dot = gps1ve
            E2_dot = gps2ve - (d * X_hat[5][0] * math.sin(X_hat[2][0]) + d * math.cos(X_hat[2][0]))
            N1_dot = gps1vn
            N2_dot = gps2vn - (d * X_hat[5][0] * math.cos(X_hat[2][0]) - d * math.sin(X_hat[2][0]))
            psi_gnss1=math.atan2((E1_dot),(N1_dot))
            psi_gnss2=math.atan2((E2_dot),(N2_dot))
            H_GNSS=np.array([[0, 1, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0],[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]])
            Z_GNSS=np.array([[E1], [E2], [N1], [N2], [psi_gnss1], [N1_dot], [N2_dot], [E1_dot], [E2_dot]])
            V_GNSS=np.array([4e-0, 4e-0, 4e-0, 4e-0, 8e-0, 5e-0, 8e-0, 5e-0, 8e-0])
            # print(f"GPS 1 pos: {N1} {E1}, vel: {N1_dot} {E1_dot}")
            # print(f"GPS 2 pos: {N2} {E2}, vel: {N2_dot} {E2_dot}")
            #[X_hat, P_hat] = SubscribeAndPublish.kalman_filter_update(X_hat, P_hat, Z_GNSS, H_GNSS, V_GNSS, I)
      



        X = X_hat[0][0]
        Y = X_hat[1][0]
        psi = X_hat[2][0]
        omega=X_hat[2][0]
        vel = math.sqrt(X_hat[3][0]**2 + X_hat[4][0]**2)
        rate = X_hat[5][0]
        # print(f"x: {X_hat[0][0]}")
        # print(f"y: {X_hat[1][0]}")
        # print(f"psi: {X_hat[2][0]}")
        # print(f"dx: {X_hat[3][0]}")
        # print(f"dy: {X_hat[4][0]}")
        # print(f"dpsi: {X_hat[5][0]}")
        # print("")

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