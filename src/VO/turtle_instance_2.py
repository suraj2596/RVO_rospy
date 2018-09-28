#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from turtlesim.srv import TeleportAbsolute

from std_msgs.msg import String
from rvo.msg import Information

from random import randint
import numpy as np
import time

class TurtleBot:

    #global all_agents_pose_dict
    #all_agents_pose_dict = {}

    def __init__(self, agent_name):

		# Creates a node with name of the agent
        self.agent_name = agent_name
        print(self.agent_name)
        rospy.init_node("x")

		### Publisher ###

		# Publisher which will publish velocity to the topic '/turtleX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/'+self.agent_name+'/cmd_vel', Twist, queue_size=10)

        # Publisher which will publish Pose to the topic '/turtleX/cmd_vel'.
        self.turtle_teleport = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

        # Publisher which will publish to the topic '/common_information'
        self.publish_information = rospy.Publisher("/common_information", Information, queue_size=10)

        self.pub_pose = Pose()
        self.inf = Information()

        ### Subscriber ###

        # self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/'+self.agent_name+'/pose', Pose, self.update_pose)

        # self.recieve_from_information_channel is called when a message of type information is received.
        rospy.Subscriber("/common_information", Information, self.recieve_from_information_channel)

        self.pose = Pose()

        self.rate = rospy.Rate(10)

#-----------------------------------------------------------------------------------------#
# Functions related to topics
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


    def publish_to_information_channel(self,t):
        i = Information()
        i.agent_name = t
        i.agent_pose_x = self.pose.x
        i.agent_pose_y = self.pose.y
        i.agent_heading = self.heading


        self.publish_information.publish(i)
        print("Published the turtle node name on topic /common_information")
        self.rate.sleep()

    def recieve_from_information_channel(self,data):
        self.inf = data
        self.name_temp = self.inf.agent_name
        self.x_temp = self.inf.agent_pose_x
        self.y_temp = self.inf.agent_pose_y
        self.heading_temp = self.inf.agent_heading

        if(self.name_temp == self.agent_name):
            self.pose_updated = [self.x_temp, self.y_temp, self.heading_temp]
            #all_agents_pose_dict.update({self.name_temp: self.pose_updated})


        #print("Recieved x is : %f" %self.x_temp)
        #should store values in dictonary
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# Helper functions
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    # sets heading towards given direction
    def set_heading(self,theta):
        #theta_rad = (theta * np.pi)/180
        rospy.sleep(0.1)
        self.turtle_teleport(self.pose.x,self.pose.y,theta)

    # sets heading towards given co-ordinates
    def set_goal_heading(self,x,y):
        rospy.sleep(0.1)
        self.heading = atan2(y - self.pose.y, x - self.pose.x)
        self.turtle_teleport(self.pose.x,self.pose.y,self.heading)
        #print(self.heading)
        return

    # Returns position and velocity of given agent
    #def get_agent_information(self,turtle_name):

    # Returns True when called if the agent is on collision course
    def collision(self, turtle_name=0,r=2):
        #self.x_other_agent,self.y_other_agent = get_agent_information(turtle_name)
        return True

    def choose_new_velocity(self):
        return 1.5

# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# RVO functions
    def move2goal_rvo(self,x,y):
        self.goal_pose = Pose()

        # Get the input from the function call.
        self.goal_pose.x = x
        self.goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.2

        # Setting the velocity
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 3
        self.set_goal_heading(x,y)
        self.velocity_publisher.publish(self.vel_msg)

        #rospy.sleep(10)
        while self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            if(self.collision() != True):
                #vel_msg.linear.x = np.min(rvo[0]-self.heading,rvo[-1]-self.heading)+self.heading
                #self.heading = self.rvo[np.where(np.min(abs(np.subtract(self.rvo,self.heading))))]
                print("Inside RVO. Should choose new velocity")
                print("The new choosen velocity is : ")
                #self.heading = self.choose_new_velocity()
                #print(self.heading)
                #self.set_heading(self.heading)
            else:
                self.set_goal_heading(x,y)
                self.velocity_publisher.publish(self.vel_msg)

            self.publish_to_information_channel(self.agent_name)
            #print(all_agents_pose_dict)
            rospy.sleep(0.1)

        # Stopping the agent after the movement is over.
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# Testing functions
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
# end
#-----------------------------------------------------------------------------------------#


if __name__ == '__main__':
    try:
        x1 = TurtleBot("turtle2")
        x1.move2goal_rvo(8,1)

    except rospy.ROSInterruptException:
        pass
