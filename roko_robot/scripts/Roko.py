#!/usr/bin/env python3

import math, random, numpy
import matplotlib.pyplot as plt
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)

class Roko:
    # Class wide variables
    # Robot constant parameters
    _mass = 12 # robot mass, kg
    _rw = 0.1 # wheels radius, m
    _lw = 0.5 # length of the wheelset, m
    _lx = 1.2 # length of the robot, m
    _ly = 0.6 # width of the robot, m
    _lz = 0.3 # height of the robot, m
    _mw = 0.3 # wheel mass, kg
    # Moments of inertia
    _Jw = 0.5 * _mw * _rw**2
    _Jd = 0.25 * _mw * _rw**2 + 1/12 *_mw * 0.04**2
    _Jx = 0.25 * _mass * _ly**2 + 1/12 * _mass * _lz**2
    _Jy = 0.25 * _mass * _lx**2 + 1/12 * _mass * _lz**2
    _Jz = 0.5 * _mass * _ly**2

    # Robot motion parameters
    _x = 0
    _y = 0
    _heading = 0 # rad
    _velocity = 0
    _angular_rate = 0 # rad/s
    _last_acc = 0

    # Robot control get_list_values
    _Ml = 0
    _Mr = 0

    # Robot PID control variables
    _velocity_error = 0
    _rate_error = 0

    # System variables
    _dt = 0.01
    _random_seed = 0
    _gyro_offset = 0.0
    _gps1_con = 0.0
    _gps2_con = 0.0
    _gps1_last_pos = [0, 0]
    _gps2_last_pos = [0, 0]
    _gps2_seed = 0.0
    _gps1_seed = 0.0

    # Arrays to store motion parameters at every simulation step
    _X = []
    _Y = []
    _Heading = []
    _Velocity = []
    _Angular_rate = []
    _Time = []


    def update(self): #-> 'do things'
        # Rotation motion equation
        angular_acceleration = (self._Mr - self._Ml) * self._lw / self._rw \
        / (2*self._Jd + self._Jz + 2*self._lw**2*self._Jw/self._rw**2 \
        + 2*self._mw*self._lw**2)

        # Linear motion equation
        linear_acceleration = (self._Ml + self._Mr) * (self._Jy \
        + self._lz**2*self._mass - self._rw*self._lw*self._mass) \
        / (2*self._Jw/self._rw*(self._Jy + self._lz**2*self._mass) \
        + 2*self._Jy*self._rw*(0.5*self._mass + self._mw) \
        + 2*self._lz**2*self._rw*self._mass*self._mw)

        # TODO: add lateral motion
        self._last_acc = linear_acceleration

        # Progress model parameters forward
        self._heading = self._heading + self._angular_rate * self._dt;
        self._x = self._x + self._velocity * math.cos(self._heading) * self._dt;
        self._y = self._y + self._velocity * math.sin(self._heading) * self._dt;
        # Note that coordinate change is not instant
        # Velocity changes first and coordinates will be changed on the next simulation step
        self._angular_rate = self._angular_rate + angular_acceleration * self._dt;
        self._velocity = self._velocity + linear_acceleration * self._dt;

        # Limit heading angle to [-pi, pi] interval
        if abs(self._heading) > math.pi:
            self._heading = self._heading - numpy.sign(self._heading) * 2 * math.pi

        self._random_seed = self._random_seed + 1

        # Collect logs for futher visualization
        self._X.append(self._x)
        self._Y.append(self._y)
        self._Heading.append(self._heading * 180.0 / math.pi) # store in degrees
        self._Velocity.append(self._velocity)
        self._Angular_rate.append(self._angular_rate)
        if len(self._Time) == 0:
            self._Time.append(self._dt)
        else:
            self._Time.append(self._Time[-1] + self._dt)


    def __init__(self, x, y, heading, velocity, angular_rate):
        # Set initial conditions
        self._x = x
        self._y = y
        self._heading = heading # rad
        self._velocity = velocity
        self._angular_rate = angular_rate # rad/s

        # Clear the log arrays
        self._X = []
        self._Y = []
        self._Heading = []
        self._Velocity = []
        self._Angular_rate = []
        self._Time = []

        self._gyro_offset = random.gauss(0, 2 * math.pi / 180.0)
        self._gps1_con = random.gauss(0, 1)
        self._gps2_con = random.gauss(0, 1)


    def set_motion(self, velocity, angular_rate):
        '''Set desired linear velocity and angular rate for the robot
        to obtain necessary control values for the wheel motors.
        Keep in mind, that control functions also use measured parameters.
        With errors!!!'''
        params = self.get_measurements()
        # Velocity PD controller
        velocity_error = velocity - params[0]
        velocity_control = 6 * velocity_error + 0.3 * (velocity_error - self._velocity_error)
        self._velocity_error = velocity_error
        # Angular rate PD controller
        rate_error = angular_rate - params[1]
        rate_control = 2.8 * rate_error + 0.1 * (rate_error - self._rate_error)
        self._rate_error = rate_error

        self._Ml = (velocity_control - rate_control) / 2
        self._Mr = (velocity_control + rate_control) / 2


    def get_odometry(self):
        readings = [0, 0]
        # Left wheel
        readings[0] = (2 * self._velocity + 2 * self._lw * self._angular_rate) / 2 / self._rw
        # Right wheel
        readings[1] = (2 * self._velocity - 2 * self._lw * self._angular_rate) / 2 / self._rw

        # Apply measurement noise
        odo_noise = 0.2 # rad/sec
        readings[0] += random.gauss(0, odo_noise)
        readings[1] += random.gauss(0, odo_noise)

        return readings


    def get_imu(self):
        readings = [0, 0, 0, 0, 0, 0]
        readings[0] = 0.0  # gyro X
        readings[1] = 0.0  # gyro Y
        readings[2] = self._angular_rate  # gyro.Z

        readings[3] = self._last_acc - 2 * readings[2] * 0.0
        readings[4] = 2 * readings[2] * self._velocity
        readings[5] = 0.0

        # Apply measurement noise
        gyro_noise = 3 * math.pi / 180.0  # rad
        accl_noise = 0.4  # m/s2
        readings[0] += random.gauss(0, gyro_noise)
        readings[1] += random.gauss(0, gyro_noise)
        readings[2] += self._gyro_offset + random.gauss(0, gyro_noise)

        readings[3] += random.gauss(0, accl_noise)
        readings[4] += random.gauss(0, accl_noise)
        readings[5] += random.gauss(0, accl_noise)

        return readings


    def get_gps(self):
        readings = [0, 0, 0, 0, 0, 0, 0, 0]

        # The first GPS receiver is located just between the wheels
        readings[0] = self._x
        readings[1] = self._y

        # Apply measurement noise
        pos_mu = 0.01
        pos_noise = 0.7
        vel_noise = 0.2
        gps1_dseed = - pos_mu * self._gps1_seed + math.sqrt(2 * pos_noise ** 2 * pos_mu) * (random.random() - 0.5);
        self._gps1_seed +=  + gps1_dseed * self._dt;

        # X measurement
        readings[0] += self._gps1_seed + self._gps1_con * math.sin(self._heading) + random.gauss(0, pos_noise)
        # Y measurement
        readings[1] += self._gps1_seed + self._gps1_con * math.cos(self._heading) + random.gauss(0, pos_noise)

        # North velocity measurement
        readings[2] = (readings[0] - self._gps1_last_pos[0]) / self._dt
        # East velocity measurement
        readings[3] = (readings[1] - self._gps1_last_pos[1]) / self._dt

        # Store current position measurements
        self._gps1_last_pos = readings[0:2]


        # The second GPS receiver is located jat the back of the Robot
        # The displacement along Xb axis is -0.4 m
        lx = -0.4
        readings[4] = self._x + lx * math.cos(self._heading)
        readings[5] = self._y + lx * math.sin(self._heading)
        #readings[6] = (self._velocity + self._angular_rate * lx) * cos(self._heading)
        #readings[7] = (self._velocity + self._angular_rate * lx) * sin(self._heading)

        # Apply measurement noise
        pos_mu = 0.03
        pos_noise = 0.7
        vel_noise = 0.2
        gps2_dseed = - pos_mu * self._gps2_seed + math.sqrt(2 * pos_noise ** 2 * pos_mu) * (random.random() - 0.5);
        self._gps2_seed +=  + gps2_dseed * self._dt;

        # X measurement
        readings[4] += self._gps2_seed + self._gps2_con * math.sin(self._heading) + random.gauss(0, pos_noise)
        # Y measurement
        readings[5] += self._gps2_seed + self._gps2_con * math.cos(self._heading) + random.gauss(0, pos_noise)

        # North velocity measurement
        readings[6] = (readings[4] - self._gps2_last_pos[0]) / self._dt
        # East velocity measurement
        readings[7] = (readings[5] - self._gps2_last_pos[1]) / self._dt

        # Store current position measurements
        self._gps2_last_pos = readings[4:6]

        return readings


    def get_measurements(self):
        '''Get measured values for all motion parameters.
        Remember - on real robot you can never know exact
        values for these parameters.This function simulates just that.'''
        # Factory parameters for sensor errors
        gyro_shift = 5 * math.pi/180 # rad/s
        gyro_noise = 2.5 * math.pi/180 # rad/s
        gnss_shift = 2 # m
        gnss_noise = 0.2 # m
        odo_scale = 0.08 # percent
        odo_noise = 0.03 # m/s

        # Long-period errors of position measurement
        #position_shift_x = 0.6*gnss_shift*math.cos(self._random_seed/900 + random.random()*0.2)
        #position_shift_y = random.random()*0.2 - 0.1 + 0.4*gnss_shift*math.sin(self._random_seed/1350 + random.random()*0.1)
        #if (self._random_seed/1000) % 2 < 1:
        #    position_shift_x = 0.7*gnss_shift*math.sin(self._random_seed/1200 + random.random()*0.1)
        #    position_shift_y = random.random()*0.3 - 0.15 + 0.6*gnss_shift*math.cos(self._random_seed/900 + random.random()*0.2)

        #heading_shift = gyro_shift*math.cos((self._random_seed - 250)/190)

        # Measurements
        #x = self._x + position_shift_x + gnss_noise * (random.random() - 0.5)
        #y = self._y + position_shift_y + gnss_noise * (random.random() - 0.5)
        #heading = self._heading + heading_shift + gyro_noise * (random.random() - 0.5)
        velocity = (self._velocity + odo_noise * (random.random() - 0.5)) * (1 + 2 * odo_scale * (random.random() - 0.5))
        angular_rate = (self._angular_rate + 0.15 * gyro_noise * (random.random() - 0.5)) * (1 + 2 * odo_scale * (random.random() - 0.5))

        return [velocity, angular_rate]


    def plot_results(self):
        gridsize = (3,2)
        fig = plt.figure(figsize=(13,9.3))
        ax1 = plt.subplot2grid(gridsize,(0,0), rowspan=2)
        ax2 = plt.subplot2grid(gridsize,(2,0))
        ax3 = plt.subplot2grid(gridsize,(0,1))
        ax4 = plt.subplot2grid(gridsize,(1,1))
        ax5 = plt.subplot2grid(gridsize,(2,1))
        # Plot trajectory
        ax1.set_title('Robot trajectory')
        ax1.plot(self._X, self._Y, 'b', linewidth=2)
        #ax1.plot(trX, trY, '--k', linewidth=2)
        ax1.set_aspect('equal', adjustable='box')
        ax1.grid()
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        # Plot X and Y graphs
        ax2.plot(self._Time, self._X, 'b', linewidth=2, label='X')
        ax2.plot(self._Time, self._Y, 'r', linewidth=2, label='Y')
        ax2.grid()
        ax2.set_xlabel('Time, s')
        ax2.set_ylabel('Coordinates, m')
        ax2.legend()
        # Plot Heading graph
        ax3.plot(self._Time, self._Heading, 'g', linewidth=2)
        ax3.grid()
        ax3.set_ylabel('Heading angle, deg')
        ax3.yaxis.set_major_locator(MultipleLocator(90))
        # Plot Angular rate graph
        ax4.plot(self._Time, self._Angular_rate, 'b', linewidth=2)
        ax4.grid()
        ax4.set_ylabel('Angular rate, rad/s')
        # Plot Velocity graph
        ax5.plot(self._Time, self._Velocity, 'r', linewidth=2)
        ax5.grid()
        ax5.set_xlabel('Time, s')
        ax5.set_ylabel('Velocity, m/s')
        # Display all plots
        #plt.show()
