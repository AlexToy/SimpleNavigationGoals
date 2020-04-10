#!/usr/bin/env python
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class SimpleNavigationGoals():

    def __init__(self):
        rospy.init_node('simple_navigation_goals')


    def go_to(self, x, y, theta):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0
        q = quaternion_from_euler(0.0, 0.0, theta)
        goal.target_pose.pose.orientation = Quaternion(*q)

        rospy.loginfo("sending goal pose "+str(goal.target_pose.pose.position.x)+
        ", "+str(goal.target_pose.pose.position.y)+", "+str(goal.target_pose.pose.orientation)+ " to action server")
        client.send_goal(goal)
        print("new goal")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available !")
            rospy.signal_shutdown("Action server not available !")
        else:
            return client.get_result()

    
    def _shutdown(self):
        rospy.signal_shutdown("Shutdown")

