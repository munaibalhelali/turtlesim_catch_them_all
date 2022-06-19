#!/usr/bin/env python3
from urllib import request
import rospy 
from turtlesim_catch_them_all.msg import Turtle, ArrayTurtle
from std_msgs.msg import String
from turtlesim.srv import Kill, Spawn
import random 
import math 

caught_turtle_topic = '/caught'
active_turtles_topic = '/active_turtles'
class TurtleSpawner:
    def __init__(self):
        self.num_active_turtles = 0
        self.max_active_turtles = 10
        self.active_turtles = []

        #subscribers
        self.caught_turtle_sub = rospy.Subscriber(
                                    caught_turtle_topic,
                                    String,
                                    self.caught_turtle_callback,
                                    queue_size=10)
        #publishers
        self.active_turtles_publisher = rospy.Publisher(
                                            active_turtles_topic,
                                            ArrayTurtle,
                                            queue_size=10)
        self.spawn_main_turtle()
        #timers
        rospy.Timer(rospy.Duration(0.5), self.publish_active_turtles)
        rospy.Timer(rospy.Duration(3), self.spawn_new_turtle)
    
    def spawn_main_turtle(self):
        
        x = random.uniform(1.0, 11.0)
        y = random.uniform(1.0, 11.0)
        theta = random.uniform(0.0, math.pi)
        name = 'main_turtle'

        rospy.wait_for_service('/spawn', 5)
                # rospy.logwarn('Waiting for Spawn service to be up ...')
        
        try:
            spawn_turtle_client = rospy.ServiceProxy('/spawn', Spawn)
            response = spawn_turtle_client(x=x, y=y, theta=theta, name=name)
            rospy.loginfo(f'New turtle was created: {response.name}')

        except rospy.ServiceException as e:
            print(f'Service call failed: {e}')

    def spawn_new_turtle(self, event):
        if self.num_active_turtles > self.max_active_turtles:
            return 
        
        x = random.uniform(1.0, 11.0)
        y = random.uniform(1.0, 11.0)
        theta = random.uniform(0.0, math.pi)

        rospy.wait_for_service('/spawn', 5)
                # rospy.logwarn('Waiting for Spawn service to be up ...')
        
        try:
            spawn_turtle_client = rospy.ServiceProxy('/spawn', Spawn)
            response = spawn_turtle_client(x=x, y=y, theta=theta)
            turtle = Turtle()
            turtle.name = response.name 
            turtle.x = x 
            turtle.y = y 
            turtle.theta = theta
            self.active_turtles.append(turtle)
            rospy.loginfo(f'New turtle was created: {turtle.name}')
            self.num_active_turtles += 1
        except rospy.ServiceException as e:
            print(f'Service call failed: {e}')


    def publish_active_turtles(self, event):
        if len(self.active_turtles) >0:
            array_turtles = ArrayTurtle()
            for turtle in self.active_turtles:
                array_turtles.active_turtles.append(turtle)
            try:
                self.active_turtles_publisher.publish(array_turtles)
            except rospy.exceptions.ROSSerializationException as e:
                rospy.logwarn(f'Could not publish the active turtles: {e}')


    #callbacks                             
    def caught_turtle_callback(self, msg):
        active_turtles = [t.name for t in self.active_turtles]
        try:
            turtle_idx = active_turtles.index(msg.data)
            self.active_turtles.pop(turtle_idx)
            rospy.wait_for_service('kill', 5)
                # rospy.logwarn('Waiting for Kill service to be up ...')
            
            try:
                kill_turtle_client = rospy.ServiceProxy('kill', Kill)


                kill_turtle_client.call(name=msg.data)
                self.num_active_turtles -= 1
                rospy.loginfo(f'Turtle {msg.data} was killed!')

            except rospy.ServiceException as e:
                print(f'Service call failed: {e}')

        except ValueError as e:
            rospy.loginfo(f'The caught turtle {msg.data} is not active')

def main(*args):
    rospy.init_node('turtle_spawner', anonymous=True)
    spawner = TurtleSpawner()
    rospy.spin()


if __name__ == '__main__':
    main()
    