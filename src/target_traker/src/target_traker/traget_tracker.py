#!/usr/bin/python
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg._Obstacles import Obstacles
from simple_navigation_goals import simple_navigation_goals


def callback(msg):
    circle_target_x = msg.circles[0].center.x
    circle_target_y = msg.circles[0].center.y
    circle_target_z = msg.circles[0].center.z
    print(msg.circles[0])

    nav_goals.go_to(circle_target_x - 0.25, circle_target_y - 0.25, circle_target_z)


rospy.init_node('scan_values')
sub = rospy.Subscriber('/raw_obstacles', Obstacles, callback)
nav_goals = simple_navigation_goals.SimpleNavigationGoals()

rospy.spin()