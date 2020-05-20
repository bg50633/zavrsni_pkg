#!/usr/bin/env python

import rospy
import numpy as np
from scipy import linalg
from scipy.spatial.transform import Rotation
from math import *

from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import tf


class HLC():

    def __init__(self):
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # Initialize all class variables used in self.odom_sub_callback
        self.p_meas_x = 0
        self.p_meas_y = 0
        self.p_meas_z = 0

        self.v_meas_x = 0
        self.v_meas_y = 0
        self.v_meas_z = 0

        self.q_w_meas = 1
        self.q_x_meas = 0
        self.q_y_meas = 0
        self.q_z_meas = 0
        self.odom_sub = rospy.Subscriber('mavros/global_position/local', Odometry, self.odom_sub_callback)

        # Initialize all class variables used in self.trajectory_sub_callback
        self.trajectory_started = False

        self.p_ref_x = 0
        self.p_ref_y = 0
        self.p_ref_z = 0

        self.v_ref_x = 0
        self.v_ref_y = 0
        self.v_ref_z = 0

        self.a_ref_x = 0
        self.a_ref_y = 0
        self.a_ref_z = 0

        self.w_ref_x = 0
        self.w_ref_y = 0
        self.w_ref_z = 0

        self.q_w_ref = 1
        self.q_x_ref = 0
        self.q_y_ref = 0
        self.q_z_ref = 0
        self.trajectory_sub = rospy.Subscriber('mavros/trajectory', MultiDOFJointTrajectoryPoint, self.trajectory_sub_callback)

        self.attitude = AttitudeTarget()
        self.R_des = Quaternion()
        self.W_des = Vector3()
        self.C_cmd = 0.


        self.K_pos = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.K_vel = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
        self.D = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]])

        self.a_g = np.array([0, 0, 9.81])
        self.k_h = 0.009

        self.uav_mass = 0.5
        self.max_rpm = 1000
        self.motor_constant = 8.54858e-06
        self.f_max = self.motor_constant * self.max_rpm ** 2
        self.C_max = self.f_max / self.uav_mass
        print(self.C_max)


    def odom_sub_callback(self, data):
        # dohvacanje mjerenih vrijednosti pozicije
        self.p_meas_x = data.pose.pose.position.x
        self.p_meas_y = data.pose.pose.position.y
        self.p_meas_z = data.pose.pose.position.z

        # dohvacanje mjerenih vrijednosti brzine
        self.v_meas_x = data.twist.twist.linear.x
        self.v_meas_y = data.twist.twist.linear.y
        self.v_meas_z = data.twist.twist.linear.z

        # dohvacanje mjerenih vrijednosti orijentacije
        self.q_w_meas = data.pose.pose.orientation.w
        self.q_x_meas = data.pose.pose.orientation.x
        self.q_y_meas = data.pose.pose.orientation.y
        self.q_z_meas = data.pose.pose.orientation.z


    def trajectory_sub_callback(self, data):
        self.trajectory_started = True

        # postavljanje referentnih vrijednosti pozicije
        self.p_ref_x = data.transforms[0].translation.x
        self.p_ref_y = data.transforms[0].translation.y
        self.p_ref_z = data.transforms[0].translation.z

        # postavljanje referentnih vrijednosti brzine
        self.v_ref_x = data.velocities[0].linear.x
        self.v_ref_y = data.velocities[0].linear.y
        self.v_ref_z = data.velocities[0].linear.z

        # postavljanje referentnih vrijednosti akceleracije
        self.a_ref_x = data.accelerations[0].linear.x
        self.a_ref_y = data.accelerations[0].linear.y
        self.a_ref_z = data.accelerations[0].linear.z

        # postavljanje referentnih vrijednosti kutne brzine
        self.w_ref_x = data.velocities[0].angular.x
        self.w_ref_y = data.velocities[0].angular.y
        self.w_ref_z = data.velocities[0].angular.z

        # postavljanje referentnih vrijednosti orijentacije
        self.q_w_ref = data.transforms[0].rotation.w
        self.q_x_ref = data.transforms[0].rotation.x
        self.q_y_ref = data.transforms[0].rotation.y
        self.q_z_ref = data.transforms[0].rotation.z


    # transformacije
    def quaternion2euler(self, qw, qx, qy, qz):
        phi = atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        the = asin(2 * (qw * qy - qx * qz))
        ksi = atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
        return phi, the, ksi

    def euler2matrix(self, phi, the, ksi):
        R11 = cos(ksi)*cos(the)
        R12 = cos(ksi)*sin(the)*sin(phi) - sin(ksi)*cos(phi)
        R13 = cos(ksi)*sin(the)*cos(phi) + sin(ksi)*sin(phi)
        R21 = sin(ksi)*cos(the)
        R22 = sin(ksi)*sin(the)*sin(phi) + cos(ksi)*cos(phi)
        R23 = sin(ksi)*sin(the)*cos(phi) - cos(ksi)*sin(phi)
        R31 = -sin(the)
        R32 = cos(the)*sin(phi)
        R33 = cos(the)*cos(phi)
        R = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
        return R


    def matrix2euler(self, R):
        sy = sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6

        if  not singular :
            x = atan2(R[2,1] , R[2,2])
            y = atan2(-R[2,0], sy)
            z = atan2(R[1,0], R[0,0])
        else :
            x = atan2(-R[1,2], R[1,1])
            y = atan2(-R[2,0], sy)
            z = 0

        return np.array([x, y, z])

    def euler2quaternion(self, phi, the, ksi):
        qw = cos(phi/2)*cos(the/2)*cos(ksi/2) + sin(phi/2)*sin(the/2)*sin(ksi/2)
        qx = sin(phi/2)*cos(the/2)+cos(ksi/2) - cos(phi/2)*sin(the/2)*sin(ksi/2)
        qy = cos(phi/2)*sin(the/2)*cos(ksi/2) + sin(phi/2)*cos(the/2)*sin(ksi/2)
        qz = cos(phi/2)*cos(the/2)*sin(ksi/2) - sin(phi/2)*sin(the/2)*cos(ksi/2)
        return qw, qx, qy, qz



    def calculate_reference_values(self):
        # formiranje varijabli reference
        self.p_ref = np.array([self.p_ref_x, self.p_ref_y, self.p_ref_z])
        self.v_ref = np.array([self.v_ref_x, self.v_ref_y, self.v_ref_z])
        self.a_ref = np.array([self.a_ref_x, self.a_ref_y, self.a_ref_z])
        self.w_ref = np.array([self.w_ref_x, self.w_ref_y, self.w_ref_z])

        # phi, the, ksi = self.quaternion2euler(self.q_w_ref, self.q_x_ref, self.q_y_ref, self.q_z_ref)
        #self.R_ref = self.euler2matrix(phi, the, ksi)
        reference_quaternion = [self.q_x_ref, self.q_y_ref, self.q_z_ref, self.q_w_ref]
        self.R_ref = tf.transformations.quaternion_matrix(reference_quaternion)[:3,:3]
        euler_angles = tf.transformations.euler_from_quaternion(reference_quaternion)
        self.heading = euler_angles[2]

        np.set_printoptions(suppress=True)
        print("Referent rotation: ")
        print(self.R_ref)


    def calculate_measured_values(self):
        # formiranje varijabli povratne veze
        self.p_meas = np.array([self.p_meas_x, self.p_meas_y, self.p_meas_z])
        self.v_meas = np.array([self.v_meas_x, self.v_meas_y, self.v_meas_z])
        
        #phi, the, ksi = self.quaternion2euler(self.q_w_meas, self.q_x_meas, self.q_y_meas, self.q_z_meas)
        #self.R_meas = self.euler2matrix(phi, the, ksi)
        self.R_meas = tf.transformations.quaternion_matrix(
            [self.q_x_meas, self.q_y_meas, self.q_z_meas, self.q_w_meas]
        )[:3,:3]

        np.set_printoptions(suppress=True)
        print("Measured rotation: ")
        print(self.R_meas)


    def calculate_a_des(self):
        # racunanje     a_fb = K_pos*(p_ref - p_meas) + K_vel*(v_ref - v_meas)
        self.a_fb = np.dot(self.K_pos, (self.p_ref - self.p_meas)) + np.dot(self.K_vel, (self.v_ref - self.v_meas))     # (48)
        # racunanje     a_rd = -R_ref * D * R_ref^T * v_ref
        self.a_rd = -1*np.dot(np.dot(np.dot(self.R_ref, self.D), self.R_ref.T), self.v_ref)     # (49)
        # racunanje     a_des = a_fb + a_ref - a_rd + a_g
        self.a_des = self.a_fb + self.a_ref - self.a_rd + self.a_g      # (47)


    def calculate_R_des(self):
        x_c = np.array([cos(self.heading), sin(self.heading), 0])       # (16)
        y_c = np.array([-sin(self.heading), cos(self.heading), 0])      # (17)

        z_des = self.a_des/linalg.norm(self.a_des)                          # (50)
        x_des = np.cross(y_c, z_des)/linalg.norm(np.cross(y_c, z_des))      # (51)
        y_des = np.cross(z_des, x_des)                                      # (52)
        R_des = np.array([x_des, y_des, z_des]).T                           # (53)
        
        np.set_printoptions(suppress=True)
        print("Desired rotation: ")
        print(R_des)

        phi, the, ksi = self.matrix2euler(R_des)
        q = tf.transformations.quaternion_from_euler(phi, the, ksi)
        self.R_des.w = q[3]
        self.R_des.x = q[0]
        self.R_des.y = q[1]
        self.R_des.z = q[2]


    def calculate_W_des(self):
        W_des = np.dot(np.dot(self.R_meas.T,self.R_ref),self.w_ref)     # (55)
        self.W_des.x = W_des[0]
        self.W_des.y = W_des[1]
        self.W_des.z = W_des[2]


    def calculate_C_cmd(self):
        r_ref = self.R_ref.T        # (21)
        x_b = r_ref[0]              # (18)
        y_b = r_ref[1]              # (19)
        z_b = r_ref[2]              # (20)
        C_cmd = np.dot(self.a_des, z_b)  - self.k_h*(np.dot(self.v_meas,(x_b+y_b))) ** 2       # (54)
        self.C_cmd = C_cmd / self.C_max


    def run(self):
        while not rospy.is_shutdown():
            if self.trajectory_started:
                self.calculate_reference_values()
                self.calculate_measured_values()

                self.calculate_a_des()
                self.calculate_R_des()
                self.calculate_W_des()
                self.calculate_C_cmd()

                self.attitude.orientation = self.R_des
                self.attitude.body_rate = self.W_des
                self.attitude.thrust = self.C_cmd
                
                self.attitude_pub.publish(self.attitude)
                print("\n\n")
                rospy.sleep(0.02)


if __name__=='__main__':
    rospy.init_node('HLC_reg')
    try:
        N = HLC()
        N.run()
    except rospy.ROSInterruptException:
        pass