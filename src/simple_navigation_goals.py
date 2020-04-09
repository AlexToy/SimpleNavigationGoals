#!/usr/bin/env python
import rospy
import actionlib #Brings in the SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # Brings in the .action file and messages used by the move base action

class SimpleNavigationGoals():

    def __init__(self):
    
        rospy.init_node('simple_navigation_goals')

        # Create an action client called "move_base" with action definition file "MoveBaseAction
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available !")
            rospy.signal_shutdown("Action server not available !")
            return
        rospy.loginfo("Connecting to move base server")
        rospy.loginfo("Starting goals achievements")


    def go_to(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = theta
        rospy.loginfo("sending goal pose "+str(goal.target_pose.pose.position.x)+
        ", "+str(goal.target_pose.pose.position.y)+", "+str(goal.target_pose.pose.position.z)+ "to action server")
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available !")
            rospy.signal_shutdown("Action server not available !")
        else:
            return self.client.get_result()
        