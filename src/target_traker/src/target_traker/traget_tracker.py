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
TARGET_FAKE_RADIUS = TARGET_RADIUS + 0.2
TARGET_MOVE_SAFETY_MARGIN = 1.7 #Use to know if it's the good target
TARGET_SAME_POS_SAFETY_MARGIN = 0.1

state = 0

class Target():

    def __init__(self):
        self.target_radius = TARGET_RADIUS
        self.current_target_pos_x = 0
        self.current_target_pos_y = 0
        self.init_state = True
        self._not_move = False
        self.target_move_count = 0
        self.start_timer_test_move = 0
        self.current_timer_test_move = 0
        self.timer_test_move = 0
        self.test_target_pos_x = 0
        self.test_target_pos_y = 0
        self.target_pos_sub = rospy.Subscriber(
            '/raw_obstacles', Obstacles, self.target_callback
        )

    
    def target_not_move(self, new_pos_x, new_pos_y):
        self.current_timer_test_move = time.time()
        #First call, set the timer and the target position
        #Return false because we don't know if the target has moved
        if self.target_move_count == 0:
            self.start_timer_test_move = time.time()
            self.test_target_pos_x = self.current_target_pos_x
            self.test_target_pos_y = self.current_target_pos_y
            self.target_move_count = 1
            #print("Target_not_move() : First call")
            return False

        #Second call after 3sec
        elif (self.target_move_count == 1 and
            (self.current_timer_test_move - self.start_timer_test_move) >= 3.0):

            #if the target has not moved, return true, reset the counter and the timer for the next call
            if (new_pos_x <= self.test_target_pos_x + TARGET_SAME_POS_SAFETY_MARGIN and
                new_pos_x >= self.test_target_pos_x - TARGET_SAME_POS_SAFETY_MARGIN and
                new_pos_y <= self.test_target_pos_y + TARGET_SAME_POS_SAFETY_MARGIN and
                new_pos_y >= self.test_target_pos_y - TARGET_SAME_POS_SAFETY_MARGIN):
                self.target_move_count = 0
                self.timer_test_move = 0
                #print("Target_not_move() : Second call, target not move")
                return True

            #If the target has moved, return false, reset the counter and the timer for the next call
            else:
                self.target_move_count = 0
                self.timer_test_move = 0
                #print("Target_not_move() : Second call, target move")
                return False
        
        else:
            #Timer has not arrived to 3sec, return false
            return False


    def target_callback(self, msg):
        #Target not find
        if not msg.circles:
            #print("target not find")
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
            #print("target found !")
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
                        #Target move ?
                        self._not_move = self.target_not_move(msg.circles[0].center.x, msg.circles[0].center.y)
                        #Set the new target position
                        self.current_target_pos_x = msg.circles[0].center.x
                        self.current_target_pos_y = msg.circles[0].center.y
                        #print("Target is correct, set new position !")
                        return
                    #print("Target y position is not possible")
                    return
                #print("Target x position is not possible")
                #print("x pos = ", msg.circles[0].center.x)
                #print(" + = ", self.current_target_pos_x + TARGET_MOVE_SAFETY_MARGIN)
                #print(" - = ", self.current_target_pos_y - TARGET_MOVE_SAFETY_MARGIN)
                return
            #print("Target radius is not possible")
            #print("true_radius = ", msg.circles[0].true_radius)
            #print(" + = ", TARGET_RADIUS + TARGET_RADIUS_SAFETY_MARGIN)
            #print(" - = ", TARGET_RADIUS - TARGET_RADIUS_SAFETY_MARGIN)
            return

        #Many targets found
        elif len(msg.circles) > 1 and self.init_state == False:
            #print(len(msg.circles), "Targets found")
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
                            #Target move ?
                            self._not_move = self.target_not_move(msg.circles[try_target].center.x, msg.circles[try_target].center.y)
                            #Set the new target position
                            self.current_target_pos_x = msg.circles[try_target].center.x
                            self.current_target_pos_y = msg.circles[try_target].center.y
                            #print("Target ", try_target, " is correct, set new position !")
                            return
                        #print("Target ", try_target, " y position is not possible")
                        return
                    #print("Target ", try_target, " x position is not possible")
                    return
                #print("Target ", try_target, " radius is not possible")
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
        self.current_robot_pos_x = 0
        self.current_robot_pos_y = 0
    
    def robot_feedback(self, msg):
        #add the current position to the robot position list
        data = msg.feedback.base_position.pose
        self.pos_x.append(data.position.x)
        self.pos_y.append(data.position.y)
        self.current_robot_pos_x = data.position.x
        self.current_robot_pos_y = data.position.y


def new_target_coordonates(target_x, target_y, robot_x, robot_y):
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    hypotenuse = math.sqrt(delta_x*delta_x + delta_y*delta_y) - (math.sqrt(delta_x*delta_x + delta_y*delta_y) - TARGET_FAKE_RADIUS)
    theta = math.atan2(delta_x,delta_y)
    new_x = target_x + math.sin(theta)*hypotenuse
    new_y = target_y + math.cos(theta)*hypotenuse

    return new_x, new_y


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

            nav_goals.go_to((target.current_target_pos_x - 0.35), (target.current_target_pos_y - 0.35), 0, 60, "map", False)

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
            print("robot x,y :", robot.current_robot_pos_x,robot.current_robot_pos_y)
            print("target x,y :", target.current_target_pos_x,target.current_target_pos_y)
            x, y = new_target_coordonates(target.current_target_pos_x, target.current_target_pos_y, robot.current_robot_pos_x, robot.current_robot_pos_y)
            nav_goals.go_to(x, y, 0, 60, "map", False)
            if target._not_move == True:
                print("Target not move")
                #state = 2
        
        elif state == 2:
            time.sleep(1)
            if nav_goals.is_arrived() == True: 
                print("Robot has arrived near to the target")
                #if target is arrived, wait 3s and go to the initial position
                time.sleep(3)
                nav_goals.go_to(robot.initial_pos_x, robot.initial_pos_y, 0)

    rospy.spin()
                
        