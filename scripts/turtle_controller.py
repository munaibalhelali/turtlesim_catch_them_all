#!/usr/bin/env python3

import rospy
from turtlesim_catch_them_all.msg import Turtle, ArrayTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import math

active_turtles_topic = '/active_turtles'
main_turtle_cmd_vel_topic = '/main_turtle/cmd_vel'
caught_turtle_topic = '/caught'
class TurtleController:
    def __init__(self):
        self.target = None
        self.curr_pose = None
        self.main_turtle = 'main_turtle'
        self.active_turtles = []

        #control constants
        self.dist_tolerance = 0.5
        self.heading_err_tolerance = 0.5 
        self.ka = 0.5 #angular gain constant
        self.kl = 0.5 #linear gain constant 
        self.freq = 100 

        #subscribers
        self.main_turtle_pose_subscriber = rospy.Subscriber(
                                            f'/{self.main_turtle}/pose',
                                            Pose,
                                            self.main_turtle_pose_callback,
                                            queue_size=10)
        self.active_turtles_subscriber = rospy.Subscriber(
                                            active_turtles_topic, 
                                            ArrayTurtle, 
                                            self.active_turtles_callback, 
                                            queue_size=10)

        #publishers
        self.main_turtle_command_publisher = rospy.Publisher(
                                            main_turtle_cmd_vel_topic,
                                            Twist,
                                            queue_size= 10)       
        self.caught_turtles_publisher = rospy.Publisher(
                                            caught_turtle_topic,
                                            String,
                                            queue_size=10)


        #Flags 
        self.reached_goal = False 

        #others
        rospy.Timer(rospy.Duration(1/self.freq), self.control_loop)

    def control_loop(self, event):
        

        if self.target == None:
            self.set_new_target()
            if self.target == None: 
                return

        dist = self.get_distance_to_target()
        err = self.get_heading_error()
        
        if dist is None or err is None:
            return 

        if dist < self.dist_tolerance:
            rospy.loginfo('Target was caught!')
            msg = String()
            msg.data = self.target.name
            self.caught_turtles_publisher.publish(msg)
            self.target = None    
            return   

        msg = Twist()
        if abs(err) > self.heading_err_tolerance:
            msg.angular.z = self.ka*err 
        else:
            msg.linear.x = self.kl*dist 

        self.main_turtle_command_publisher.publish(msg) 
        
    #utility methods
    def set_new_target(self):
        if len(self.active_turtles) > 0:
            self.target = self.active_turtles.pop(0)
            rospy.loginfo(f'New taget: {self.target.name}')
            rospy.loginfo(f'Numbe of other active turtle: {len(self.active_turtles)}')
            self.reached_goal = False
        else:
            rospy.logwarn('No active turtles!')
        
    def get_heading_error(self):
        if self.target is None or self.curr_pose is None:
            return None

        deltaX = self.target.x - self.curr_pose.x
        deltaY = self.target.y - self.curr_pose.y
        theta = math.atan2(deltaY, deltaX)
        heading_error = theta - self.curr_pose.theta

        if abs(heading_error) > math.pi:
            heading_error -= 2*math.pi
        elif abs(heading_error) < - math.pi:
            heading_error += 2*math.pi

        return heading_error

    def get_distance_to_target(self):
        if self.target is None or self.curr_pose is None:
            return None
        curr_vec = [self.curr_pose.x, self.curr_pose.y]
        target_vec = [self.target.x, self.target.y]
        return math.dist(target_vec, curr_vec)

    #callbacks
    def main_turtle_pose_callback(self, msg):
        self.curr_pose = msg

    def active_turtles_callback(self, msg):
        for turtle in msg.active_turtles:
            if len(self.active_turtles) > 0:
                active_turtles = [t.name for t in self.active_turtles]
                if turtle.name in active_turtles:
                    continue
                self.active_turtles.append(turtle)
            else:
                self.active_turtles.append(turtle)

def main(*args):
    rospy.init_node('turtle_controller', anonymous=True)
    controller = TurtleController()
    rospy.spin()


if __name__ == '__main__':
    main()
