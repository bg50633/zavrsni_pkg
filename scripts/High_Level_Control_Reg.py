#!/usr/bin/env python

import rospy
from pid import PID
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Vector3

class HLC():

    def __init__(self):
        self.attitude_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.odom_sub = rospy.Subscriber('mavros/global_position/local', Odometry, self.odom_sub_callback)
        self.imu_sub = rospy.Subscriber('mavros/imu/data', Imu, self.imu_sub_callback)
        self.trajectory_sub = rospy.Subscriber('mavros/', MultiDOFJointTrajectoryPoint, self.trajectory_sub_callback)       # potrebno odrediti topic!

        self.odom = Odometry()
        self.imu = Imu()
        self.trajectory_point = MultiDOFJointTrajectoryPoint()

        self.trajectory_started = False


        self.x_pos_pid = PID()
        self.y_pos_pid = PID()
        self.z_pos_pid = PID()
        self.x_vel_pid = PID()
        self.y_vel_pid = PID()
        self.z_vel_pid = PID()
        self.dt_pid = 0


        self.a_des = Vector3(0., 0., 0.)
        self.a_fb = Vector3(0., 0., 0.)
        self.a_ref = Vector3(0., 0., 0.)
        self.a_g = Vector3(0., 0., -9.81)



    def odom_sub_callback(self, data):
        self.odom = data

    def imu_sub_callback(self, data):
        self.imu = data

    def trajectory_sub_callback(self, data):
        self.trajectory_point = data
        self.trajectory_started = True



    def reset_pids(self):
        self.x_pos_pid.reset()
        self.y_pos_pid.reset()
        self.z_pos_pid.reset()
        self.x_vel_pid.reset()
        self.y_vel_pid.reset()
        self.z_vel_pid.reset()

    def set_pids(self):
        self.x_pos_pid.set_kp(1)
        self.x_pos_pid.set_ki(0)
        self.x_pos_pid.set_kd(0.1)

        self.y_pos_pid.set_kp(1)
        self.y_pos_pid.set_ki(0)
        self.y_pos_pid.set_kd(0.1)

        self.z_pos_pid.set_kp(1)
        self.z_pos_pid.set_ki(0)
        self.z_pos_pid.set_kd(0.1)

        self.x_vel_pid.set_kp(1)
        self.x_vel_pid.set_ki(0)
        self.x_vel_pid.set_kd(0.1)

        self.y_vel_pid.set_kp(1)
        self.y_vel_pid.set_ki(0)
        self.y_vel_pid.set_kd(0.1)

        self.z_vel_pid.set_kp(1)
        self.z_vel_pid.set_ki(0)
        self.z_vel_pid.set_kd(0.1)





    def calculate_a_des(self):
        self.set_pids()
        dt = self.dt_pid
        self.a_ref.x = self.trajectory_point.accelerations.linear.x         # dohvacanje a_ref
        self.a_ref.y = self.trajectory_point.accelerations.linear.y
        self.a_ref.z = self.trajectory_point.accelerations.linear.z


        p_ref_x = self.trajectory_point.transforms.translation.x            # dohvacanje referenci pozicije za x, y i z
        p_ref_y = self.trajectory_point.transforms.translation.y
        p_ref_z = self.trajectory_point.transforms.translation.z

        p_meas_x = self.odom.pose.pose.position.x                           # dohvacanje povratne veze pozicije za x, y i z
        p_meas_y = self.odom.pose.pose.position.y
        p_meas_z = self.odom.pose.pose.position.z

        u_pos_x = self.x_pos_pid.compute(p_ref_x, p_meas_x, dt)             # racunanje upravljacke velicine za pozicije
        u_pos_y = self.y_pos_pid.compute(p_ref_y, p_meas_y, dt)
        u_pos_z = self.z_pos_pid.compute(p_ref_z, p_meas_z, dt)


        v_ref_x = self.trajectory_point.velocities.linear.x                 # dohvacanje referenci brzine za x, y i z
        v_ref_y = self.trajectory_point.velocities.linear.y
        v_ref_z = self.trajectory_point.velocities.linear.z

        v_meas_x = self.odom.twist.twist.linear.x                           # dohvacanje povratne veze brzine za x, y i z
        v_meas_y = self.odom.twist.twist.linear.y
        v_meas_z = self.odom.twist.twist.linear.z

        u_vel_x = self.x_vel_pid.compute(v_ref_x, v_meas_x, dt)             # racunanje upravljacke velicine za brzine
        u_vel_y = self.y_vel_pid.compute(v_ref_y, v_meas_y, dt)
        u_vel_z = self.z_vel_pid.compute(v_ref_z, v_meas_z, dt)


        self.a_fb.x = u_pos_x + u_vel_x                             # racunanje a_fb
        self.a_fb.y = u_pos_y + u_vel_y
        self.a_fb.z = u_pos_z + u_vel_z

        self.a_des.x = self.a_fb.x + self.a_ref.x + self.a_g.x      # racunanje a_des
        self.a_des.y = self.a_fb.y + self.a_ref.y + self.a_g.y
        self.a_des.z = self.a_fb.z + self.a_ref.z + self.a_g.z


    def calculate_R_des(self):


    def calculate_C_cmd(self):


    def calculate_W_des(self):
    



    def run(self):
        self.reset_pids()
        self.set_pids()
        while not rospy.is_shutdown():
            if not self.trajectory_started:
                continue
            else:
                self.calculate_a_des()
                self.calculate_R_des()
                self.calculate_C_cmd()
                self.calculate_W_des()

                self.attitude_pub.publish()
                rospy.sleep(1.0)


            




if __name__=='__main__':
    rospy.init_node('HLC_reg')
    try:
        N = HLC()
        N.run()
    except rospy.ROSInterruptException:
        pass