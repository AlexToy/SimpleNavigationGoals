#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(msg.ranges)


rospy.init_node('scan_values')
lidar = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()