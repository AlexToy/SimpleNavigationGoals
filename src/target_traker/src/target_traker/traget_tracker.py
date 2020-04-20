#!/usr/bin/python
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg._Obstacles import Obstacles
from simple_navigation_goals import simple_navigation_goals
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import Twist

#Constantes
TARGET_RADIUS = 0.2
TARGET_RADIUS_SAFETY_MARGIN = 0.1
TARGET_MOVE_SAFETY_MARGIN = 0.25

state = 0

class Target():

    def __init__(self):
        self.target_x = 0
        self.target_y = 0
        self.target_radius = TARGET_RADIUS
        self.current_target_pos_x = 0
        self.current_target_pos_y = 0
        self.init_state = True
        self.target_pos_sub = rospy.Subscriber(
            '/raw_obstacles', Obstacles, self.target_callback
        )

    def target_callback(self, msg):
        print("ok")
        #Target not find
        if not msg.circles:
            print("target not find")
            return

        #Initial target : Set the initial target pos 
        elif ( self.init_state == True and
            len(msg.circles) == 1 and 
            msg.circles[0].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
            msg.circles[0].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN) ):

            self.current_target_pos_x = msg.circles[0].center.x
            self.current_target_pos_y = msg.circles[0].center.y
            self.init_state = False
            print("inital target position ...")
            print("inital position x = ", self.current_target_pos_x)
            print("inital position y = ", self.current_target_pos_y)
            return

        #Target found
        elif len(msg.circles) == 1 and self.init_state == False:
            print("target found !")
            #Radius is possible ?
            if (msg.circles[0].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
                msg.circles[0].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN)):
                #Target x position is possible ?
                if (msg.circles[0].center.x <= (self.current_target_pos_x + TARGET_MOVE_SAFETY_MARGIN) and
                    msg.circles[0].center.x >= (self.current_target_pos_x - TARGET_MOVE_SAFETY_MARGIN)):
                    #Target y position is possible ?
                    if (msg.circles[0].center.y <= (self.current_target_pos_y + TARGET_MOVE_SAFETY_MARGIN) and
                        msg.circles[0].center.y >= (self.current_target_pos_y - TARGET_MOVE_SAFETY_MARGIN)):
                        #Target is the correct !
                        self.target_x = msg.circles[0].center.x
                        self.target_y = msg.circles[0].center.y
                        self.current_target_pos_x = msg.circles[0].center.x
                        self.current_target_pos_y = msg.circles[0].center.y
                        print("Target is correct, set new position !")
                        return
                    print("Target y position is not possible")
                    return
                print("Target x position is not possible")
                print("x pos = ", msg.circles[0].center.x)
                print(" + = ", self.current_target_pos_x + TARGET_MOVE_SAFETY_MARGIN)
                print(" - = ", self.current_target_pos_y - TARGET_MOVE_SAFETY_MARGIN)
                return
            print("Target radius is not possible")
            print("true_radius = ", msg.circles[0].true_radius)
            print(" + = ", TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN)
            print(" - = ", TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN)
            return

        #Many targets found
        elif len(msg.circles) > 1 and self.init_state == False:
            print(len(msg.circles), "Targets found")
            #Finding the right target
            for try_target in range(len(msg.circles)-1):
                #Radius is possible ?
                if (msg.circles[try_target].true_radius <= (TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN) and 
                    msg.circles[try_target].true_radius >= (TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN)):
                    #Target x position is possible ?
                    if (msg.circles[try_target].center.x <= (self.current_target_pos_x + TARGET_MOVE_SAFETY_MARGIN) and
                        msg.circles[try_target].center.x >= (self.current_target_pos_x - TARGET_MOVE_SAFETY_MARGIN)):
                        #Target y position is possible ?
                        if (msg.circles[try_target].center.y <= (self.current_target_pos_y + TARGET_MOVE_SAFETY_MARGIN) and
                            msg.circles[try_target].center.y >= (self.current_target_pos_y - TARGET_MOVE_SAFETY_MARGIN)):
                            #Target is the correct !
                            self.target_x = msg.circles[try_target].center.x
                            self.target_y = msg.circles[try_target].center.y
                            self.current_target_pos_x = msg.circles[try_target].center.x
                            self.current_target_pos_y = msg.circles[try_target].center.y
                            print("Target ", try_target, " is correct, set new position !")
                            return
                        print("Target ", try_target, " y position is not possible")
                        return
                    print("Target ", try_target, " x position is not possible")
                    return
                print("Target ", try_target, " radius is not possible")
                return


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
        #add the current position to the robot position list
        data = msg.feedback.base_position.pose
        self.pos_x.append(data.position.x)
        self.pos_y.append(data.position.y)


if __name__ == "__main__":
    rospy.init_node('scan_values')
    nav_goals = simple_navigation_goals.SimpleNavigationGoals()
    target = Target()
    robot = Robot()
    twist = Twist()

    while True:
        #time.sleep(0.1)
        if state == 0:
            while target.init_state == True:
                None
            print("Target initialization done !")

            nav_goals.go_to((target.current_target_pos_x - 0.21), (target.current_target_pos_y - 0.21), 0)

            time.sleep(1)

            if not robot.pos_x:
                print("error with robot pos")
            else:
                robot.initial_pos_x = robot.pos_x[0]
                robot.initial_pos_y = robot.pos_y[0]
                print("added initial pos of robot")
                state = 1
            
        elif state == 1:
            time.sleep(2)
            print("Go to the target")
            nav_goals.go_to((target.current_target_pos_x - 0.21), (target.current_target_pos_y - 0.21), 0)

    rospy.spin()
                
        