#!/usr/bin/python
import rospy
import math
from sensor_msgs.msg import LaserScan



def callback(msg):
    i=0
    for _ in msg.ranges:
        i = i +1
    print(i)
    print("#####################################--FIN--##########################################")
    


rospy.init_node('scan_values')
lidar = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()