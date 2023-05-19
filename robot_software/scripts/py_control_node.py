#!/usr/bin/env python3

import rospy
import math
import numpy as np
from roko_robot.msg import control
from roko_robot.msg import navigation


class SubscribeAndPublish:

    #coordinates = [[0,0],[4,0],[8,4],[8,8],[4,12],[0,12],[-4,8],[-4,4],[0,0]] # [x,y]
    coordinates = [[0,0],[4,0],[8,4],[8,8],[4,12],[0,13],[0,17],[4,21],[8,21]] # [x,y]
    # coordinates = [[0,0],[8,0],[16,8],[16,16],[8,24],[0,24],[0,32],[8,40],[16,40]] # [x,y]
    #coordinates = [[1,1],[16,0],[16,16],[0,16],[1,1]] # [x,y]
    indexOfCoor = 1
    nextCoordinate = coordinates[indexOfCoor]
    prevCoordinate = coordinates[indexOfCoor - 1]

    prevInt = 0
    prevDiff = 0
    prevDIST = 8
    prevV = 0

    prevIntFI = 0
    prevFI = 0

    crossTrackError = 0

    def __init__(self):

        # Create publisher for publishing sensor measurements
        self._navigationSub = rospy.Subscriber('roko/navigation_data', navigation, self.navigation_callback)
        self._controlPub = rospy.Publisher('roko/control_data', control, queue_size=10)

    # This function will be called each time a new navigation message is received.
    # So basicly - every time you calculated your navigation solution in navigation_node

    def PID_V (self, DIST):
        currentDist = DIST

        V = 0
        dt = 10
        #Kp = 0.9
        #Ki = 0.00004
        # ----------
        # Kp = 2
        # Ki = 0.00001
        Kp = 4
        Ki = 0.00008
        
        Kd = 1000


        Int = SubscribeAndPublish.prevInt +  DIST * dt
        if (DIST < 0.65):
            SubscribeAndPublish.prevInt = 0
        else:
            SubscribeAndPublish.prevInt = Int

        Dif = (DIST - SubscribeAndPublish.prevDIST) / dt
        

        V = (Kp*DIST + Ki * Int + Kd*Dif)
        SubscribeAndPublish.prevV = V

        if (V > 24):
            V = 24

        return V

    def PID_FI (self,rate):
        maxRate = 2.0
        dt = 10
        # Kp = 1.5
        # Ki = 0.00004
        Kp = 5.0
        Ki = 0.0006
        Kd = 1000
        # def bound(low, high, value):
        #     return max(low, min(high, value))

        # diffCTE = (newCTE - SubscribeAndPublish.crossTrackError) / dt
        # SubscribeAndPublish.crossTrackError = newCTE

        # rate = bound(-maxRate, maxRate, 2.3 * SubscribeAndPublish.crossTrackError + 0.9 * diffCTE)

        Int = SubscribeAndPublish.prevIntFI +  rate * dt
        if (rate < 0.03):
            SubscribeAndPublish.prevIntFI = 0
        else:
            SubscribeAndPublish.prevIntFI = Int

        Dif = (rate - SubscribeAndPublish.prevFI) / dt

        fi = (Kp*rate + Ki * Int + Kd*Dif)
        #fi = (Kp*rate + Ki * Int + Kd*Dif + 6*diffCTE)

        if (fi > 2):
            fi = 2

        rospy.loginfo("fi{:.2f}".format(fi))
        return fi



    def navigation_callback(self, msg):
        # Extract navigation values from the message
        timestamp = msg.timestamp
        x = msg.X
        y = msg.Y
        omega = msg.Omega


        # rospy.loginfo("omega {:.2f}".format(omega))
        if omega > 2*np.pi :
            K = int(omega / 2*np.pi)
            omega = omega - K * 2*np.pi





        coordinates = SubscribeAndPublish.coordinates
        nextCoordinate = SubscribeAndPublish.nextCoordinate
        prevCoordinate = SubscribeAndPublish.prevCoordinate

        

        if ( x < nextCoordinate[0] + 0.6 and x > nextCoordinate[0] - 0.6 and y < nextCoordinate[1] + 0.6 and y > nextCoordinate[1] - 0.6):
            SubscribeAndPublish.prevCoordinate = nextCoordinate

            SubscribeAndPublish.indexOfCoor += 1

            if (SubscribeAndPublish.indexOfCoor > len(SubscribeAndPublish.coordinates) - 1):
                print("STOP")
                self.publishControl(0, 0)
                return



            SubscribeAndPublish.nextCoordinate = coordinates[SubscribeAndPublish.indexOfCoor]
            SubscribeAndPublish.prevCoordinate = coordinates[SubscribeAndPublish.indexOfCoor - 1]

        nextCoordinate = SubscribeAndPublish.nextCoordinate
        prevCoordinate = SubscribeAndPublish.prevCoordinate


        deltaX = (nextCoordinate[0] - x)
        deltaY = (nextCoordinate[1] - y)

        # prevDeltaX = (x - prevCoordinate[0])
        # prevDeltaY = (y - prevCoordinate[1])

        # testX = nextCoordinate[0] - prevCoordinate[0]
        # testY = nextCoordinate[1] - prevCoordinate[1]

        # newCTE = (prevDeltaY * testX - prevDeltaX * testY) / (testX**2 + testY**2)

        DIST = (deltaX**2 + deltaY**2)**(1/2)
        V = self.PID_V(DIST)
        #rospy.loginfo("[control] V {:.2f}".format(V))
        SubscribeAndPublish.prevDIST = DIST


        newOmega = omega



        if (deltaX != 0):
            rospy.loginfo("deltaY {:.2f} deltaX{:.2f}".format(deltaY,deltaX))
            newOmega = math.atan2(deltaY,deltaX)

            LEFT = False

            # if (newOmega < -np.pi + 0.1 and newOmega > -np.pi -0.2):
            #     newOmega = np.pi

            # if (newOmega < -2 and newOmega > -2.8):
            #     newOmega = abs(newOmega) + np.pi/2

            rospy.loginfo("newOmega {:.2f} omega{:.2f} LEFT {:.2f}".format(newOmega,omega,LEFT))

            rate = newOmega - omega

            # if (rate < 0 and newOmega - omega <= -3.14):
            #     rate = newOmega + omega

            # if (rate < 0 and rate < -np.pi):
            #     rate = newOmega + omega
            # elif (rate > 0 and rate >=np.pi):
            #     rate = rate  np.pi

            #rospy.loginfo("rate {:.2f}".format(rate))

            SubscribeAndPublish.prevFI = abs(rate)

            if ( abs(rate) < 0.15):
                left_speed = V
                right_speed = V
            else:
                if ( rate) <= 0 :
                    a = self.PID_FI(abs(rate))
                    # left_speed = -a
                    # right_speed = a

                    # left_speed = -a + V
                    # right_speed = V

                    left_speed = V
                    right_speed = -a +V




                    # if (V <= 24 and V > 8):
                    #     left_speed = -a
                    #     right_speed = a
                    
                else :
                    a = self.PID_FI(abs(rate))
                    # left_speed = a
                    # right_speed = -a

                    # right_speed = -a + V
                    # left_speed = V

                    right_speed = V
                    left_speed = -a +V

                    # if (V <= 24 and V > 8):
                    #     right_speed = -a
                    #     left_speed = a
                    
        else:
            left_speed = V
            right_speed = V


        # Send control values to the robot
        self.publishControl(left_speed, right_speed)


    def publishControl(self, left_speed, right_speed):
        if abs(left_speed) < 6:
            left_speed = math.copysign(6, left_speed)
        if abs(right_speed)  < 6:
            right_speed  = math.copysign(6, right_speed)
        msg = control()
        msg.timestamp = int(rospy.get_time() * 1000)
        msg.left = left_speed  # rotation speed of the left wheel
        msg.right = right_speed  # rotation speed of the right wheel

        self._controlPub.publish(msg)
        #print("Control command {:.2f} {:.2f} published".format(left_speed, right_speed))

# End of SubscribeAndPublish class


def main():
    rospy.init_node('py_control_node')
    SAP = SubscribeAndPublish()

    rospy.spin()
# End on main()


if __name__ =="__main__":
    main()
