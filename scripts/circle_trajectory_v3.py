#!/usr/bin/env python

import rospy
from math import cos
from math import sin
from math import pi
from math import atan2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class Node():
    def __init__(self):
        self.pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber('mavros/local_position/odom', Odometry, self.sub_callback)

        self.odom = Odometry()
        self.pose = PoseStamped()
        self.q = Quaternion()

        self.r = 1
        self.ang = 5
        self.trajectory = []


    def sub_callback(self, data):
        self.odom = data


    def calculate_trajectory(self):
        r = self.r
        ang = self.ang
        l = []
        for i in range(360/ang):
            x = r*cos(i*ang*pi/180)
            y = r*sin(i*ang*pi/180)
            pom = [x,y]
            t = tuple(pom)
            l.append(t)
        self.trajectory = l


    def euler2quaternion(self, phi, the, ksi):
        self.q.w = cos(phi/2)*cos(the/2)*cos(ksi/2) + sin(phi/2)*sin(the/2)*sin(ksi/2)
        self.q.x = sin(phi/2)*cos(the/2)+cos(ksi/2) - cos(phi/2)*sin(the/2)*sin(ksi/2)
        self.q.y = cos(phi/2)*sin(the/2)*cos(ksi/2) + sin(phi/2)*cos(the/2)*sin(ksi/2)
        self.q.z = cos(phi/2)*cos(the/2)*sin(ksi/2) - sin(phi/2)*sin(the/2)*cos(ksi/2)



    def run(self):
        self.calculate_trajectory()
        i=0
        while not rospy.is_shutdown():
            if i >= 360/self.ang:
                i = 0

            pose_x = self.trajectory[i][0]
            pose_y = self.trajectory[i][1]
            pose_z = self.odom.pose.pose.position.z

            phi = 0
            the = 0
            ksi = (atan2(pose_y,pose_x) + pi)

            self.euler2quaternion(phi, the, ksi)
            
            self.pose.pose.position.x = pose_x
            self.pose.pose.position.y = pose_y
            self.pose.pose.position.z = pose_z
            self.pose.pose.orientation = self.q

            i += 1
            self.pub.publish(self.pose)
            rospy.sleep(1.0)


if __name__=='__main__':
    rospy.init_node('circle_trajectory')
    try:
        N = Node()
        N.run()
    except rospy.ROSInterruptException:
        pass