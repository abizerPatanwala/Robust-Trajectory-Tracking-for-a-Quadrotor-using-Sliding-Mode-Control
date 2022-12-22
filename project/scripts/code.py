#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#!/usr/bin/env python3
import math
from math import pi, sqrt, atan2, cos, sin
#from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os


def find_traj(t0, tf, q):
    A = [
        [1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
         ]
    a_coeff = np.dot(np.linalg.inv(A),q)
    return a_coeff
    
class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",
        Odometry, self.odom_callback, queue_size=1)
        rospy.Rate(100)
        
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.xd = 0
        self.yd = 0
        self.zd = 0
        self.xdot = 0
        self.ydot = 0
        self.zdot = 0
        self.xddot = 0
        self.yddot = 0
        self.zddot = 0
        self.xd_series = [];
        self.mutex_lock_on = False
        
        # TODO: include initialization codes if needed
        '''
        self.ax0 = [0,0,0,0,0,0]
        self.ax1 = [-0.5802, 0.3951, -0.09887, 0.0109, -4.9389e-04, 7.9012e-06]
        self.ax2 = [1, 0, 0, 0, 0, 0]
        self.ax3 = [987.6543, -120.9877, 5.8765, -0.1412, 0.0017, -7.9012e-06]
        self.ax4 = [0, 0, 0, 0, 0, 0]
       
        
        self.ay0 = [0, 0, 0, 0, 0, 0]
        self.ay1 = [0, 0, 0, 0, 0, 0]
        self.ay2 = [-96.3951, 19.3580, -1.5210, 0.0583, -0.0011, 7.9012e-06]
        self.ay3 = [1, 0, 0, 0, 0, 0]
        self.ay4 = [4692.358, -417.2840, 14.7654, -0.2598, 0.0023, -7.9012e-06 ]
        
        self.az0 = [0, 0, 0, 0.08, -0.024, 0.00192]
        self.az1 = [1, 0, 0, 0, 0, 0]
        self.az2 = [1, 0, 0, 0, 0, 0]
        self.az3 = [1, 0, 0, 0, 0, 0]
        self.az4 = [1, 0, 0, 0, 0, 0]
        '''
        
        t0 = 0
        tf = 5
        qx = [0,0,0,0,0,0]
        qy = [0,0,0,0,0,0]
        qz = [0,0,0,1,0,0]
        self.ax0 = find_traj(t0, tf, qx)
        self.ay0 = find_traj(t0, tf, qy)
        self.az0 = find_traj(t0, tf, qz)

        t0 = 5
        tf = 20
        qx = [0,0,0,1,0,0]
        qy = [0,0,0,0,0,0]
        qz = [1,0,0,1,0,0]
        self.ax1 = find_traj(t0, tf, qx)
        self.ay1 = find_traj(t0, tf, qy)
        self.az1 = find_traj(t0, tf, qz)
        
        t0 = 20
        tf = 35
        qx = [1,0,0,1,0,0]
        qy = [0,0,0,1,0,0]
        qz = [1,0,0,1,0,0]
        self.ax2 = find_traj(t0, tf, qx)
        self.ay2 = find_traj(t0, tf, qy)
        self.az2 = find_traj(t0, tf, qz)

        t0 = 35
        tf = 50
        qx = [1,0,0,0,0,0]
        qy = [1,0,0,1,0,0]
        qz = [1,0,0,1,0,0]
        self.ax3 = find_traj(t0, tf, qx)
        self.ay3 = find_traj(t0, tf, qy)
        self.az3 = find_traj(t0, tf, qz)

        t0 = 50
        tf = 65
        qx = [0,0,0,0,0,0]
        qy = [1,0,0,0,0,0]
        qz = [1,0,0,1,0,0]
        self.ax4 = find_traj(t0, tf, qx)
        self.ay4 = find_traj(t0, tf, qy)
        self.az4 = find_traj(t0, tf, qz)
        
        self.m = 0.027;
        self.g = 9.81;
        self.Ix = 16.571710*10**(-6); self.Iy = 16.571710*10**(-6); self.Iz = 29.261652*10**(-6); self.Ip = 12.65625*10**(-8);
        self.kf = 1.28192*10**(-8);
        self.km = 5.964552*10**(-3);
        self.l = 0.046;
        
        self.u_temp1 = 0
        self.u_temp2 = 0
        self.u_temp3 = 0
        self.u_temp4 = 0
        
        rospy.on_shutdown(self.save_data)
    
    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 
        # to return the desired positions, velocities and accelerations
        if self.t <= 5:
            self.zddot = 2*self.az0[2] + 6*self.az0[3]*self.t + 12*self.az0[4]*self.t**2 + 20*self.az0[5]*self.t**3
            self.zdot = self.az0[1] + 2*self.az0[2]*self.t + 3*self.az0[3]*self.t**2 + 4*self.az0[4]*self.t**3 + 5*self.az0[5]*self.t**4
            self.zd = self.az0[0] + self.az0[1]*self.t + self.az0[2]*self.t**2 + self.az0[3]*self.t**3 + self.az0[4]*self.t**4 + self.az0[5]*self.t**5
            self.xd = self.ax0[0] + self.ax0[1]*self.t + self.ax0[2]*self.t**2 + self.ax0[3]*self.t**3 + self.ax0[4]*self.t**4 + self.ax0[5]*self.t**5
            self.xdot = self.ax0[1] + 2*self.ax0[2]*self.t + 3*self.ax0[3]*self.t**2 + 4*self.ax0[4]*self.t**3 + 5*self.ax0[5]*self.t**4
            self.xddot = 2*self.ax0[2] + 6*self.ax0[3]*self.t + 12*self.ax0[4]*self.t**2 + 20*self.ax0[5]*self.t**3
            self.yd = self.ay0[0] + self.ay0[1]*self.t + self.ay0[2]*self.t**2 + self.ay0[3]*self.t**3 + self.ay0[4]*self.t**4 + self.ay0[5]*self.t**5
            self.ydot = self.ay0[1] + 2*self.ay0[2]*self.t + 3*self.ay0[3]*self.t**2 + 4*self.ay0[4]*self.t**3 + 5*self.ay0[5]*self.t**4
            self.yddot = 2*self.ay0[2] + 6*self.ay0[3]*self.t + 12*self.ay0[4]*self.t**2 + 20*self.ay0[5]*self.t**3
            
        if self.t > 5 and self.t <= 20:
            self.zddot = 2*self.az1[2] + 6*self.az1[3]*self.t + 12*self.az1[4]*self.t**2 + 20*self.az1[5]*self.t**3
            self.zdot = self.az1[1] + 2*self.az1[2]*self.t + 3*self.az1[3]*self.t**2 + 4*self.az1[4]*self.t**3 + 5*self.az1[5]*self.t**4
            self.zd = self.az1[0] + self.az1[1]*self.t + self.az1[2]*self.t**2 + self.az1[3]*self.t**3 + self.az1[4]*self.t**4 + self.az1[5]*self.t**5
            self.xd = self.ax1[0] + self.ax1[1]*self.t + self.ax1[2]*self.t**2 + self.ax1[3]*self.t**3 + self.ax1[4]*self.t**4 + self.ax1[5]*self.t**5
            self.xdot = self.ax1[1] + 2*self.ax1[2]*self.t + 3*self.ax1[3]*self.t**2 + 4*self.ax1[4]*self.t**3 + 5*self.ax1[5]*self.t**4
            self.xddot = 2*self.ax1[2] + 6*self.ax1[3]*self.t + 12*self.ax1[4]*self.t**2 + 20*self.ax1[5]*self.t**3
            self.yd = self.ay1[0] + self.ay1[1]*self.t + self.ay1[2]*self.t**2 + self.ay1[3]*self.t**3 + self.ay1[4]*self.t**4 + self.ay1[5]*self.t**5
            self.ydot = self.ay1[1] + 2*self.ay1[2]*self.t + 3*self.ay1[3]*self.t**2 + 4*self.ay1[4]*self.t**3 + 5*self.ay1[5]*self.t**4
            self.yddot = 2*self.ay1[2] + 6*self.ay1[3]*self.t + 12*self.ay1[4]*self.t**2 + 20*self.ay1[5]*self.t**3
            
        if self.t > 20 and self.t <= 35: 
            self.zddot = 2*self.az2[2] + 6*self.az2[3]*self.t + 12*self.az2[4]*self.t**2 + 20*self.az2[5]*self.t**3
            self.zdot = self.az2[1] + 2*self.az2[2]*self.t + 3*self.az2[3]*self.t**2 + 4*self.az2[4]*self.t**3 + 5*self.az2[5]*self.t**4
            self.zd = self.az2[0] + self.az2[1]*self.t + self.az2[2]*self.t**2 + self.az2[3]*self.t**3 + self.az2[4]*self.t**4 + self.az2[5]*self.t**5
            self.xd = self.ax2[0] + self.ax2[1]*self.t + self.ax2[2]*self.t**2 + self.ax2[3]*self.t**3 + self.ax2[4]*self.t**4 + self.ax2[5]*self.t**5
            self.xdot = self.ax2[1] + 2*self.ax2[2]*self.t + 3*self.ax2[3]*self.t**2 + 4*self.ax2[4]*self.t**3 + 5*self.ax2[5]*self.t**4
            self.xddot = 2*self.ax2[2] + 6*self.ax2[3]*self.t + 12*self.ax2[4]*self.t**2 + 20*self.ax2[5]*self.t**3
            self.yd = self.ay2[0] + self.ay2[1]*self.t + self.ay2[2]*self.t**2 + self.ay2[3]*self.t**3 + self.ay2[4]*self.t**4 + self.ay2[5]*self.t**5
            self.ydot = self.ay2[1] + 2*self.ay2[2]*self.t + 3*self.ay2[3]*self.t**2 + 4*self.ay2[4]*self.t**3 + 5*self.ay2[5]*self.t**4
            self.yddot = 2*self.ay2[2] + 6*self.ay2[3]*self.t + 12*self.ay2[4]*self.t**2 + 20*self.ay2[5]*self.t**3
            
        if self.t > 35 and self.t <= 50:
            self.zddot = 2*self.az3[2] + 6*self.az3[3]*self.t + 12*self.az3[4]*self.t**2 + 20*self.az3[5]*self.t**3
            self.zdot = self.az3[1] + 2*self.az3[2]*self.t + 3*self.az3[3]*self.t**2 + 4*self.az3[4]*self.t**3 + 5*self.az3[5]*self.t**4
            self.zd = self.az3[0] + self.az3[1]*self.t + self.az3[2]*self.t**2 + self.az3[3]*self.t**3 + self.az3[4]*self.t**4 + self.az3[5]*self.t**5
            self.xd = self.ax3[0] + self.ax3[1]*self.t + self.ax3[2]*self.t**2 + self.ax3[3]*self.t**3 + self.ax3[4]*self.t**4 + self.ax3[5]*self.t**5
            self.xdot = self.ax3[1] + 2*self.ax3[2]*self.t + 3*self.ax3[3]*self.t**2 + 4*self.ax3[4]*self.t**3 + 5*self.ax3[5]*self.t**4
            self.xddot = 2*self.ax3[2] + 6*self.ax3[3]*self.t + 12*self.ax3[4]*self.t**2 + 20*self.ax3[5]*self.t**3
            self.yd = self.ay3[0] + self.ay3[1]*self.t + self.ay3[2]*self.t**2 + self.ay3[3]*self.t**3 + self.ay3[4]*self.t**4 + self.ay3[5]*self.t**5
            self.ydot = self.ay3[1] + 2*self.ay3[2]*self.t + 3*self.ay3[3]*self.t**2 + 4*self.ay3[4]*self.t**3 + 5*self.ay3[5]*self.t**4
            self.yddot = 2*self.ay3[2] + 6*self.ay3[3]*self.t + 12*self.ay3[4]*self.t**2 + 20*self.ay3[5]*self.t**3
         
        if self.t > 50 and self.t <= 65:
            self.zddot = 2*self.az4[2] + 6*self.az4[3]*self.t + 12*self.az4[4]*self.t**2 + 20*self.az4[5]*self.t**3
            self.zdot = self.az4[1] + 2*self.az4[2]*self.t + 3*self.az4[3]*self.t**2 + 4*self.az4[4]*self.t**3 + 5*self.az4[5]*self.t**4
            self.zd = self.az4[0] + self.az4[1]*self.t + self.az4[2]*self.t**2 + self.az4[3]*self.t**3 + self.az4[4]*self.t**4 + self.az4[5]*self.t**5
            self.xd = self.ax4[0] + self.ax4[1]*self.t + self.ax4[2]*self.t**2 + self.ax4[3]*self.t**3 + self.ax4[4]*self.t**4 + self.ax4[5]*self.t**5
            self.xdot = self.ax4[1] + 2*self.ax4[2]*self.t + 3*self.ax4[3]*self.t**2 + 4*self.ax4[4]*self.t**3 + 5*self.ax4[5]*self.t**4
            self.xddot = 2*self.ax4[2] + 6*self.ax4[3]*self.t + 12*self.ax4[4]*self.t**2 + 20*self.ax4[5]*self.t**3
            self.yd = self.ay4[0] + self.ay4[1]*self.t + self.ay4[2]*self.t**2 + self.ay4[3]*self.t**3 + self.ay4[4]*self.t**4 + self.ay4[5]*self.t**5
            self.ydot = self.ay4[1] + 2*self.ay4[2]*self.t + 3*self.ay4[3]*self.t**2 + 4*self.ay4[4]*self.t**3 + 5*self.ay4[5]*self.t**4
            self.yddot = 2*self.ay4[2] + 6*self.ay4[3]*self.t + 12*self.ay4[4]*self.t**2 + 20*self.ay4[5]*self.t**3        
            return
   
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding trajectories
        self.traj_evaluate()
        self.xd_series.append(self.xd)
        #print("Current time is: ", self.t)
        
        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        # parametrize lambda values for control input equations
        #define tuning parameters K for ur terms in each control
        k1 = 15; k2 = 100; k3 = 100; k4 = 20;
        lambda1 = 10; lambda2 = 15; lambda3 = 1 ;lambda4 = 10
        #tuning for Fx, Fy
        kpx = 120; kdx = 5;
        kpy = 80; kdy = 5;
        boundary1 = 1; boundary2 = 0.25; boundary3 = 0.25; boundary4 = 0.25
        #if (self.u_temp1 == 0 and self.u_temp2 == 0 and self.u_temp3 == 0 and self.u_temp4 == 0):
        #    bigomega = 0;
        #else:
        allocation_matrix = [[1/(4*self.kf), -1*(2**(1/2))/(4*self.kf*self.l), -1*2**(1/2)/(4*self.kf*self.l), -1 /(4*self.km*self.kf)], 
                    [1/(4*self.kf), -1*2**(1/2)/(4*self.kf*self.l), 2**(1/2)/(4*self.kf*self.l), 1 /(4*self.km*self.kf)], 
                    [1/(4*self.kf), 2**(1/2)/(4*self.kf*self.l), 2**(1/2)/(4*self.kf*self.l), -1 /(4*self.km*self.kf)],
                    [1/(4*self.kf), 2**(1/2)/(4*self.kf*self.l), -1*2**(1/2)/(4*self.kf*self.l), 1 /(4*self.km*self.kf)]]
        controls = np.zeros([4,1])
        controls[0] = self.u_temp1
        controls[1] = self.u_temp2
        controls[2] = self.u_temp3
        controls[3] = self.u_temp4
        controls = np.asarray(controls)
        omega_sqr = np.dot(allocation_matrix, controls)
        
        if (omega_sqr[0] < 0):
            omega_sqr[0] = 0
        if (omega_sqr[1] < 0):
            omega_sqr[1] = 0
        if (omega_sqr[2] < 0):
            omega_sqr[2] = 0
        if (omega_sqr[3] < 0):
            omega_sqr[3] = 0
         
        omega = [omega_sqr[0]**(1/2), omega_sqr[1]**(1/2), omega_sqr[2]**(1/2), omega_sqr[3]**(1/2)];
        
        if omega[0] > 2618:
            omega[0] = 2618
        
        if omega[1]>2618:
            omega[1] = 2618

        if omega[2]>2618:
            omega[2] = 2618

        if omega[3]>2618:
            omega[3] = 2618
            
        bigomega = omega[0] - omega[1] + omega[2] - omega[3];
        
        #changing the rpy values to the bound -pi tp pi. need to check if the % operation works on output of tf.euler_from_matrix. it works on np array
        #rpy = rpy%math.pi
        
        
        while(rpy[0] < -math.pi ):
            rpy[0] = rpy[0] +  2 * math.pi
        while(rpy[0] > math.pi ):
            rpy[0] = rpy[0] -  2 * math.pi    
        while(rpy[1] < -math.pi):
            rpy[1] = rpy[1] +  2 * math.pi
        while(rpy[1] > math.pi ):
            rpy[1] = rpy[1] -  2 * math.pi
        while(rpy[2] < -math.pi ):
            rpy[2] = rpy[2] +  2 * math.pi
        while(rpy[2] > math.pi):
            rpy[2] = rpy[2] - 2 * math.pi
        
        
        #control for z
        c = (cos(rpy[0]) * cos(rpy[1]))/self.m;
        print("theta is: ", rpy[0])
        print("phi is: " , rpy[1])
        print("psi is: " , rpy[2])

      
        #print("rpy is: ", rpy[0], " ",rpy[1])
        #print("c is", c)
        e1 = xyz[2] - self.zd;
        
        e1dot = xyz_dot[2] - self.zdot;
        s1 = e1dot + lambda1*e1;
        if (abs(s1) < boundary1):
            sat1 = s1/ boundary1;
        else:
            sat1 = np.sign(s1)
            
        u1 = self.g/c + self.zddot/c - (lambda1 * e1dot)/c - (k1 * sat1)/c 

        Fx = self.m*(-kpx*(xyz[0]-self.xd)-kdx*(xyz_dot[0]-self.xdot) + self.xddot);
        Fy = self.m*(-kpy*(xyz[1]-self.yd) - kdy*(xyz_dot[1]-self.ydot)+ self.yddot);
        print("Fx is: ", Fx)
        print("Fy is: ", Fy)
        print("u1 is: ", u1)
        check1 = Fx/u1;
        check2 = -Fy/u1;
        if (check1 > 1):
            check1  = 1
        if (check1 < -1): 
            check1 = -1
        if (check2 > 1):
            check2  = 1
        if (check2 < -1): 
            check2 = -1

        #print('value of fx/u1 and value of -fy/u1 are', check1, check2)
        #print('force values are',Fx, Fy);
        #print('u1 values', u1);            

        #control for phi
        #need to map phi_d to -pi to pi
        phi_d = math.asin(check2);
        e2 = rpy[0] - phi_d;
        e2dot = rpy_dot[0];
        s2 = e2dot + lambda2*e2;
        if (abs(s2) < boundary2):
            sat2 = s2/ boundary2;
        else:
            sat2 = np.sign(s2)
        u2 = self.Ip* bigomega * rpy_dot[1] - self.Ix*lambda2*e2dot - rpy_dot[1] * rpy_dot[2] * (self.Iy - self.Iz) -k2 * sat2 * self.Ix;
        
  
        #control for theta;
        theta_d = math.asin(check1);
        e3 = rpy[1] - theta_d;
        
        e3dot = rpy_dot[1];
        s3 = e3dot + lambda3*e3;
        if (abs(s3) < boundary3):
            sat3 = s3/ boundary3;
        else:
            sat3 = np.sign(s3)
        u3 = -lambda3 * self.Iy * e3dot - self.Ip * bigomega * rpy_dot[0] - rpy_dot[0] * rpy_dot[2] * (self.Iz - self.Ix)  -k3 * self.Iy * sat3;
        
        #control for psi
        e4 = rpy[2];
        e4dot = rpy_dot[2];
        s4 = e4dot + lambda4*e4;
        if (abs(s4) < boundary4):
            sat4 = s4/ boundary4
        else:
            sat4 = np.sign(s4)
        u4 = -1*rpy_dot[0]*rpy_dot[1]*(self.Ix-self.Iy) - self.Iz*lambda4*e4dot - k4* sat4 *self.Iz;
        
        
        self.u_temp1 = (u1);
        self.u_temp2 = (u2);
        self.u_temp3 = (u3);
        self.u_temp4 = (u4);
        
        
        # TODO: convert the desired control inputs "u" to desired rotor
        #velocities "motor_vel" by using the "allocation matrix"
        
        #need to review this assignment done to motor_vel initially with Abizer
        motor_vel = np.zeros([4,1])
        motor_vel[0,0] = omega[0]
        motor_vel[1,0] = omega[1]
        motor_vel[2,0] = omega[2]
        motor_vel[3,0] = omega[3]
        # TODO: maintain the rotor velocities within the valid range of [0 to
        #2618]

        
        
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0], motor_vel[2,0], motor_vel[3,0]]
        self.motor_speed_pub.publish(motor_speed)
    
    
    
    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0

        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
        
    # save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        
        with open("/home/abizer-2/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.
            z_series], fp)
        '''
        fp = open("/home/abizer-2/rbe502_project/src/project/scripts/log.txt","w")
        for i in range(len(self.t_series)):
            fp.write(f"{self.t_series[i]}, {self.x_series[i]}, {self.y_series[i]}, {self.z_series[i]}\n")
        '''
            
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
