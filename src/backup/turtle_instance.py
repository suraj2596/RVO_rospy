#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from turtlesim.srv import TeleportAbsolute

from std_msgs.msg import String
from rvo.msg import information

from random import randint
import numpy as np
import time

class TurtleBot:

    def __init__(self, turtle_name):

		# Creates a node with name of the turtle
        rospy.init_node(turtle_name)

		### Publisher ###

		# Publisher which will publish velocity to the topic '/turtleX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/'+turtle_name+'/cmd_vel', Twist, queue_size=10)

        # Publisher which will publish Pose to the topic '/turtleX/cmd_vel'.
        #self.pose_publisher = rospy.Publisher('/'+turtle_name+'/teleport_absolute', Pose, queue_size=10)
        self.turtle_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

        # Publisher which will publish to the topic '/common_information'
        self.publish_information = rospy.Publisher("/common_information", information, queue_size=10)

        self.pub_pose = Pose()
        self.inf = information()
        ### Subscriber ###

        # self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/'+turtle_name+'/pose', Pose, self.update_pose)

        # self.recieve_from_information_channel is called when a message of type information is received.
        rospy.Subscriber("/common_information", information, self.r)

        self.pose = Pose()

        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def r(self,data):
        self.inf = data
        print("Recieved data is : %s" %self.inf)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=0.6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
        #return constant

    def set_heading(self,theta):
        #theta_rad = (theta * np.pi)/180
        self.turtle_teleport(self.pose.x,self.pose.y,theta)

    def set_goal_heading(self,x,y):
        #vel_msg = Twist()
        #vel_msg.linear.x = 0.1
        #self.velocity_publisher.publish(vel_msg)
        rospy.sleep(0.1)
        self.heading = atan2(y - self.pose.y, x - self.pose.x)
        self.turtle_teleport(self.pose.x,self.pose.y,self.heading)
        print(self.heading)
        return

    def publish_to_information_channel(self,turtle_name):
        i = information()
        i.turtle_name = turtle_name
        i.node_velocity.linear.x = 1
        i.node_velocity.linear.y = 1
        i.node_velocity.linear.z = 1
        i.node_velocity.angular.x = 1
        i.node_velocity.angular.y = 1
        i.node_velocity.angular.z = 1

        self.publish_information.publish(i)
        print("Published the turtle node name on topic /common_information")
        self.rate.sleep()

    def move2goal(self,x,y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = x
        goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 1

        vel_msg = Twist()

        #first set the heading
        #while current_heading != intended_heading:


        #and then travel in straight line
        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 2 #self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0 #self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()

    def update_rvo(self):
        self.rvo = np.array([0.1,1.7])
        return

    def move2goal_rvo(self,x,y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()
        # Get the input from the function call.

        goal_pose.x = x
        goal_pose.y = y
        # Please, insert a number slightly greater than 0 (e.g. 0.01).

        distance_tolerance = 0.2

        vel_msg = Twist()

        self.set_goal_heading(x,y)
        vel_msg.linear.x = 3
        self.velocity_publisher.publish(vel_msg)
        self.rvo = np.array([1,17])
        i = 0.00
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            #self.update_rvo()
            if(i>0.50):#self.rvo[0] < self.heading <self.rvo[-1]
                #vel_msg.linear.x = np.min(rvo[0]-self.heading,rvo[-1]-self.heading)+self.heading
                #self.heading = self.rvo[np.where(np.min(abs(np.subtract(self.rvo,self.heading))))]
                self.heading = 0
                self.set_heading(self.heading)
                print("Inside RVO")
            else:
                # Linear velocity in the x-axis.
                self.set_goal_heading(x,y)
            i+=0.1
            rospy.sleep(0.2)
            #print(i)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()

    def wander(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 1

        while 1:
            goal_pose.x = int(randint(0, 9))
            goal_pose.y = int(randint(0, 9))

            vel_msg = Twist()

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 3

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

    #def calc_rvo(self,)
