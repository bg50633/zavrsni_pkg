#!/usr/bin/env python

import rospy
from math import cos
from math import sin
from math import pi
from geometry_msgs.msg import PoseStamped



def calculate_coordinates(r,fi):
    l = []
    for i in range(360/fi):
        x = r*cos(i*fi*pi/180)
        y = r*sin(i*fi*pi/180)
        pom = [x,y]
        t = tuple(pom)
        l.append(t)
    return l




def circle(r,fi):
    l = calculate_coordinates(r,fi)
    i = 0
    while not rospy.is_shutdown():
        if i >= 360/fi:
            i = 0
        pose = PoseStamped()
        pose.pose.position.x = l[i][0]
        pose.pose.position.y = l[i][1]
        pose.pose.position.z = 4
        pub.publish(pose)
        i += 1
        rospy.sleep(1.0)



if __name__=='__main__':
    pub = rospy.Publisher('/red/mavros/setpoint_position/local',PoseStamped,queue_size=1)
    rospy.init_node('publisher')
    r = 1
    fi = 10
    try:
        while not rospy.is_shutdown():
            circle(r,fi)
    except rospy.ROSInterruptException:
        pass