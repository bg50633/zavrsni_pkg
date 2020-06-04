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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import tf


class HLC():

    def __init__(self):
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        self.angle_sp_pub = rospy.Publisher('uav/euler_sp', Vector3, queue_size=1)
        self.angle_mv_pub = rospy.Publisher('uav/euler_mv', Vector3, queue_size=1)
        self.angle_rate_sp_pub = rospy.Publisher('uav/angular_sp', Twist, queue_size=1)
        self.angle_rate_mv_pub = rospy.Publisher('uav/angular_mv', Twist, queue_size=1)

        self.angle_sp = Vector3()
        self.angle_mv = Vector3()
        self.angle_rate_sp = Twist()
        self.angle_rate_mv = Twist()

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

        # Initialize all class variables used in self.imu_sub_callback
        self.w_meas_x = 0
        self.w_meas_y = 0
        self.w_meas_z = 0
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_sub_callback)

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


        self.K_pos = np.array([[5.125 ,0,0],[0, 5.125, 0],[0,0, 5]])
        self.K_vel = np.array([[3.1,0,0],[0,3.1,0],[0,0,0.25]])
        
        # self.D = np.array([[0.49, 0, 0], [0, 0.49, 0], [0, 0, 0.0]])
        # self.D = np.array([[0.278, 0, 0], [0, 0.278, 0], [0, 0, 0.0]]) # Lemniscate
        self.D = np.array([[0., 0, 0], [0, 0., 0], [0, 0, 0.0]])

        self.a_g = np.array([0, 0, 9.81])
        self.k_h = 0.009

        # TODO: initialize old_heading properly
        self.old_heading = 0
        self.dt = 0.02

        self.uav_mass = 0.5
        self.max_rpm = 1070
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

    def imu_sub_callback(self, data):
        # dohvacanje mjerenih vrijednosti kutne brzine
        self.w_meas_x = data.angular_velocity.x
        self.w_meas_y = data.angular_velocity.y
        self.w_meas_z = data.angular_velocity.z

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

        reference_quaternion = [self.q_x_ref, self.q_y_ref, self.q_z_ref, self.q_w_ref]
        self.R_ref = tf.transformations.quaternion_matrix(reference_quaternion)[:3,:3]
        euler_angles = tf.transformations.euler_from_quaternion(reference_quaternion)
        self.heading = euler_angles[2]
        self.heading_rate = (self.heading - self.old_heading) / self.dt
        self.old_heading = self.heading

        d_x = self.D[0][0]
        d_y = self.D[1][1]
        d_z = self.D[2][2]

        alpha = self.a_ref + self.a_g + d_x * self.v_ref                    # (14)
        beta = self.a_ref + self.a_g + d_y * self.v_ref                      # (15)

        self.x_c = np.array([cos(self.heading), sin(self.heading), 0])       # (16)
        self.y_c = np.array([-sin(self.heading), cos(self.heading), 0])      # (17)

        x_b = np.cross(self.y_c, alpha)/linalg.norm(np.cross(self.y_c, alpha))    # (18)
        y_b = np.cross(beta, x_b)/linalg.norm(np.cross(beta, x_b))                # (19)
        z_b = np.cross(x_b, y_b)                                                 # (20)
        
        # Calculate only to get referent euler angles
        R_ref_TODO = np.array([x_b, y_b, z_b]).T                                  # (21)
        euler_angles = tf.transformations.euler_from_matrix(R_ref_TODO)

        c = np.dot(z_b, self.a_ref + self.a_g + d_z * self.v_ref)           # (22)
        # c_cmd = c - self.k_h * (np.dot(self.v_ref,(x_b + y_b))) ** 2        # (23)

        B1 = c - (d_z - d_x) * np.dot(z_b, self.v_ref)
        C1 = -(d_x - d_y) * np.dot(y_b, self.v_ref)
        D1 = d_x * np.dot(x_b, self.a_ref)
        A2 = c + (d_y - d_z) * np.dot(z_b, self.v_ref)
        C2 = (d_x - d_y) * np.dot(x_b, self.v_ref)
        D2 = -d_y * np.dot(y_b, self.a_ref)
        B3 = - np.dot(self.y_c, z_b)
        C3 = linalg.norm(np.cross(self.y_c, z_b))
        D3 = self.heading_rate * np.dot(self.x_c, x_b)

        w_x = (-B1*C2*D3 + B1*C3*D2 - B3*C1*D2 + B3*C2*D1) / (A2*(B1*C3 - B3*C1))       # (27)
        w_y = (-C1*D3 + C3*D1) / (B1*C3 - B3*C1)                                        # (28)
        w_z = (B1*D3 - B3*D1) / (B1*C3 - B3*C1)                                         # (29)

        self.w_ref = np.array([w_x, w_y, w_z])

        self.angle_sp.x = euler_angles[0]
        self.angle_sp.y = euler_angles[1]
        self.angle_sp.z = euler_angles[2]

        self.angle_rate_sp.angular.x = w_x
        self.angle_rate_sp.angular.y = w_y
        self.angle_rate_sp.angular.z = w_z

        np.set_printoptions(suppress=True)
        print("Referent rotation: ")
        print(self.R_ref)
        print("Referent heading: ")
        print(self.heading)


    def calculate_measured_values(self):
        # formiranje varijabli povratne veze
        self.p_meas = np.array([self.p_meas_x, self.p_meas_y, self.p_meas_z])
        self.v_meas = np.array([self.v_meas_x, self.v_meas_y, self.v_meas_z])
        
        measured_quaternion = [self.q_x_meas, self.q_y_meas, self.q_z_meas, self.q_w_meas]
        self.R_meas = tf.transformations.quaternion_matrix(measured_quaternion)[:3,:3]
        euler_angles = tf.transformations.euler_from_quaternion(measured_quaternion)

        self.angle_mv.x = euler_angles[0]
        self.angle_mv.y = euler_angles[1]
        self.angle_mv.z = euler_angles[2]

        self.angle_rate_mv.angular.x = self.w_meas_x
        self.angle_rate_mv.angular.y = self.w_meas_y
        self.angle_rate_mv.angular.z = self.w_meas_z

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

        np.set_printoptions(suppress=True)
        print("Desired acceleration: ")
        print(self.a_des)



    def calculate_R_des(self):
        z_des = self.a_des/linalg.norm(self.a_des)                                      # (50)
        x_des = np.cross(self.y_c, z_des)/linalg.norm(np.cross(self.y_c, z_des))        # (51)
        y_des = np.cross(z_des, x_des)                                                  # (52)
        R_des = np.array([x_des, y_des, z_des]).T                                       # (53)
        
        np.set_printoptions(suppress=True)
        print("Desired rotation: ")
        print(R_des)

        euler_angles = self.matrix2euler(R_des)
        q = tf.transformations.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])
        self.R_des.w = q[3]
        self.R_des.x = q[0]
        self.R_des.y = q[1]
        self.R_des.z = q[2]


    def calculate_W_des(self):
        W_des = np.dot(np.dot(self.R_meas.T,self.R_ref),self.w_ref)     # (55)
        self.W_des.x = W_des[0]
        self.W_des.y = W_des[1]
        self.W_des.z = W_des[2]
        print("Desired angle_rate: ")
        print(self.W_des)


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

                self.attitude.header.stamp = rospy.Time.now()
                self.attitude.orientation = self.R_des
                self.attitude.body_rate = self.W_des
                self.attitude.thrust = self.C_cmd
                self.attitude.type_mask = 7

                self.attitude_pub.publish(self.attitude)
                self.angle_sp_pub.publish(self.angle_sp)
                self.angle_mv_pub.publish(self.angle_mv)
                self.angle_rate_sp_pub.publish(self.angle_rate_sp)
                self.angle_rate_mv_pub.publish(self.angle_rate_mv)
                print("\n\n")
                rospy.sleep(self.dt)


if __name__=='__main__':
    rospy.init_node('HLC_reg')
    try:
        N = HLC()
        N.run()
    except rospy.ROSInterruptException:
        pass