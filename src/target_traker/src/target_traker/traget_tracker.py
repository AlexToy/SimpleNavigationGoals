#!/usr/bin/python
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg._Obstacles import Obstacles
#from simple_navigation_goals import simple_navigation_goals
from move_base_msgs.msg import MoveBaseActionFeedback

#Constantes
TARGET_RADIUS = 0.24
TARGET_RADIUS_SAFETY_MARGIN = 0.05
TARGET_MOVE_SAFETY_MARGIN = 0.1

state = 0

class Target():

    def __init__(self):
        self.target_x = 0
        self.target_y = 0
        self.target_radius = TARGET_RADIUS
        self.init_target_x = 0
        self.inti_target_y = 0
        self.init_state = True
        self.target_pos_sub = rospy.Subscriber(
            '/raw_obstacles', Obstacles, self.target_callback
        )

    def target_callback(self, msg):
        #Target not find
        if not msg.circles:
            print("target not find")
            return

        #Initial target
        elif ( self.init_state == True and
            len(msg.circles) == 1 and 
            msg.circles[0].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
            msg.circles[0].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN) ):

            self.init_target_x = self.target_x
            self.init_target_y = self.target_y
            self.init_state = False
            print("inital target position ...")
            return

        #Target found
        elif len(msg.circles) == 1 and self.init_state == False:
            print("target found !")
            #Radius is possible ?
            if ( msg.circles[0].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
                msg.circles[0].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN) ):
                #Target x position is possible ?
                if ( msg.circles[0].center.x <= (self.init_target_x + TARGET_MOVE_SAFETY_MARGIN) and
                    msg.circles[0].center.x >= (self.init_target_x - TARGET_MOVE_SAFETY_MARGIN) ):
                    #Target y position is possible ?
                    if ( msg.circles[0].center.y <= (self.init_target_y + TARGET_MOVE_SAFETY_MARGIN) and
                        msg.circles[0].center.y >= (self.init_target_y - TARGET_MOVE_SAFETY_MARGIN) ):
                        #Target is the correct !
                        self.target_x = msg.circles[0].center.x
                        self.target_y = msg.circles[0].center.y
                        self.init_target_x = msg.circles[0].center.x
                        self.init_target_y = msg.circles[0].center.y
                        print("Target is correct, set new position !")
                    print("Target y position is not possible")
                print("Target x position is not possible")
            print("Target radius is not possible")

        #Many targets found
        elif len(msg.circles) > 1 and self.init_state == False:
            print(len(msg.circles), "Targets found")
            #Finding the right target
            for try_target in range(len(msg.circles)):
                #Radius is possible ?
                if ( msg.circles[try_target].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
                    msg.circles[try_target].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN) ):
                    #Target x position is possible ?
                    if ( msg.circles[try_target].center.x <= (self.init_target_x + TARGET_MOVE_SAFETY_MARGIN) and
                        msg.circles[try_target].center.x >= (self.init_target_x - TARGET_MOVE_SAFETY_MARGIN) ):
                        #Target y position is possible ?
                        if ( msg.circles[try_target].center.y <= (self.init_target_y + TARGET_MOVE_SAFETY_MARGIN) and
                            msg.circles[try_target].center.y >= (self.init_target_y - TARGET_MOVE_SAFETY_MARGIN) ):
                            #Target is the correct !
                            self.target_x = msg.circles[try_target].center.x
                            self.target_y = msg.circles[try_target].center.y
                            self.init_target_x = msg.circles[try_target].center.x
                            self.init_target_y = msg.circles[try_target].center.y
                            print("Target ", try_target, " is correct, set new position !")
                        print("Target ", try_target, " y position is not possible")
                    print("Target ", try_target, " x position is not possible")
                print("Target ", try_target, " radius is not possible")


class Robot():

    def __init__(self):
        self.initial_pos_x = 0
        self.initial_pos_y = 0
        self.pos_sub = rospy.Subscriber(
            "move_base/feedback", MoveBaseActionFeedback, self.robot_feedback
        )
        self.pos_x = []
        self.pos_y = []
    
    def robot_feedback(self, msg):
        data = msg.feedback.base_position.pose
        self.pos_x.append(data.position.x)
        self.pos_y.append(data.position.y)


if __name__ == "__main__":
    rospy.init_node('scan_values')
    #nav_goals = simple_navigation_goals.SimpleNavigationGoals()
    t = Target()
    r = Robot()
    rospy.spin()

    #while True:
        #if state == 0:
            #print("State 1 : Set initial position")
            #if not r.pos_x:
                #print("list vide")
           # else:
                #r.initial_pos_x = r.pos_x[0]
                #r.initial_pos_y = r.pos_y[0]
                #print("added initial pos of robot")
                #state = 1
        
        #elif state == 1:
            #print("State 2 : follow the target")