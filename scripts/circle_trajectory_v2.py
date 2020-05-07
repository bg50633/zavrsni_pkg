#!/usr/bin/env python

import rospy
from math import cos
from math import sin
from math import pi
from math import atan2
from geometry_msgs.msg import PoseStamped



def calculate_coordinates(r,ang):
    l = []
    for i in range(360/ang):
        x = r*cos(i*ang*pi/180)
        y = r*sin(i*ang*pi/180)
        pom = [x,y]
        t = tuple(pom)
        l.append(t)
    return l


def calculate_orientation(x,y):
    return (atan2(y,x) + pi)


def euler2quaternion(phi,the,ksi):
    q_w = cos(phi/2)*cos(the/2)*cos(ksi/2) + sin(phi/2)*sin(the/2)*sin(ksi/2)
    q_x = sin(phi/2)*cos(the/2)+cos(ksi/2) - cos(phi/2)*sin(the/2)*sin(ksi/2)
    q_y = cos(phi/2)*sin(the/2)*cos(ksi/2) + sin(phi/2)*cos(the/2)*sin(ksi/2)
    q_z = cos(phi/2)*cos(the/2)*sin(ksi/2) - sin(phi/2)*sin(the/2)*cos(ksi/2)
    return q_w, q_x, q_y, q_z




def circle(r,ang):
    l = calculate_coordinates(r,ang)
    i = 0
    while not rospy.is_shutdown():
        if i >= 360/ang:
            i = 0
        pose = PoseStamped()
        pose_x = l[i][0]
        pose_y = l[i][1]
        pose.pose.position.x = pose_x
        pose.pose.position.y = pose_y
        pose.pose.position.z = 4
        
        ksi = calculate_orientation(pose_x,pose_y)
        q_w, q_x, q_y, q_z = euler2quaternion(0,0,ksi)
        pose.pose.orientation.w = q_w
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z

        pub.publish(pose)
        i += 1
        rospy.sleep(1.0)



if __name__=='__main__':
    pub = rospy.Publisher('/red/mavros/setpoint_position/local',PoseStamped,queue_size=1)
    rospy.init_node('publisher')
    r = 1
    ang = 5
    try:
        while not rospy.is_shutdown():
            circle(r,ang)
    except rospy.ROSInterruptException:
        pass