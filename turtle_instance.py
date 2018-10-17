#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from std_msgs.msg import String
from rvo.msg import Information

from random import randint
import numpy as np
import time
import os

from math import pow, atan2, sqrt, cos, sin, atan, asin

class TurtleBot:

    #global all_agents_pose_dict
    all_agents_pose_dict = {}

    def __init__(self, agent_name):
		# Creates a node with name of the agent
        self.agent_name = agent_name
        print(self.agent_name)
        rospy.init_node(self.agent_name)

		### Publisher ###

		# Publisher which will publish velocity to the topic '/turtleX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/'+self.agent_name+'/cmd_vel', Twist, queue_size=10)

        # Publisher which will publish Pose to the topic '/turtleX/cmd_vel'.
        self.turtle_teleport = rospy.ServiceProxy(self.agent_name + '/teleport_absolute', TeleportAbsolute)

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
        i.agent_vel_mag = self.vel_msg.linear.x

        #print("Published the turtle node name on topic /common_information")
        self.publish_information.publish(i)
        self.rate.sleep()

    def recieve_from_information_channel(self,data):
        self.inf = data
        self.name_temp = self.inf.agent_name
        self.x_temp = self.inf.agent_pose_x
        self.y_temp = self.inf.agent_pose_y
        self.heading_temp = self.inf.agent_heading
        self.vel_mag_temp = self.inf.agent_vel_mag

        self.pose_updated = [self.x_temp, self.y_temp, self.heading_temp, self.vel_mag_temp]
        self.all_agents_pose_dict.update({self.name_temp: self.pose_updated})
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

    def start_point(self,x,y):
        rospy.sleep(0.05)
        self.turtle_teleport(x,y,0)

    def in_VO(self,h):
        for i in self.VO:
            if(self.VO[i][0] < h[i] < self.VO[i][1]):
                return True
                break
        return False

    def in_RVO(self,h):
        print(self.RVO)
        for i in self.RVO:
            if(self.RVO[i][0] < h < self.RVO[i][1]):
                return True
                break
        return False

    def update_RVO(self,v_mag,turtle_name=0,r=2):
        rr = 2
        #calc the relative velocity of the agent and choosen other agent
        self._rel_heading = {}
        self.point_to_agent_heading = {}
        self._omega = {}
        self.VX = {}
        self.RVO = {}
        self.time_to_collision = {}
        rospy.sleep(0.01)

        self.present_temp_h = round(self.pose.theta,rr)

        #neighbouring region is a circle of 3 units
        self.NR = 3

        self._least_distance = 10

        for i in self.all_agents_pose_dict:
            if(i != self.agent_name):
                #calc distance between agent and oher agent/obstacle
                self._distance = round(sqrt(pow((self.all_agents_pose_dict[i][0] - self.pose.x), 2) + pow((self.all_agents_pose_dict[i][1] - self.pose.y), 2)),rr)

                #if it lies in the NR, consider it in calculating RVO
                if(self._distance < self.NR):
                    #calc the relative velocity of the agent and choosen other agent
                    self._rel_v_x =  v_mag * cos(self.pose.theta) - self.all_agents_pose_dict[i][3] * cos(self.all_agents_pose_dict[i][2])
                    self._rel_v_y =  v_mag * sin(self.pose.theta) - self.all_agents_pose_dict[i][3] * sin(self.all_agents_pose_dict[i][2])
                    self._rel_heading[i] = round(atan2(self._rel_v_y,self._rel_v_x),rr)

                    # VO finder :: Should output a range of headings into an 2D array
                    self.point_to_agent_heading[i] = round(atan2((self.all_agents_pose_dict[i][1] - self.pose.y),(self.all_agents_pose_dict[i][0] - self.pose.x)),rr)
                    try:
                        self._omega[i] = round(asin(r/self._distance),rr)
                    except ValueError:
                        self._omega[i] = round(np.pi/2,rr)

                    #time to collision
                    # should know distance and relative velocity in the direction of the obstacle
                    # if negative, it means collision will not occur

                    c1 = self._rel_v_x - (v_mag * (cos(self.pose.theta))) <= 0
                    c2 = self._rel_v_x - (v_mag * (cos(self.all_agents_pose_dict[i][2])))<= 0
                    c3 = self._rel_v_y - (v_mag * (sin(self.pose.theta)))<= 0
                    c4 = self._rel_v_y - (v_mag * (sin(self.all_agents_pose_dict[i][2])))<= 0

                    self.time_to_collision[i] = np.inf
                    if(c1 | c2 | c3 | c4):
                        self.time_to_collision[i] = abs(self.all_agents_pose_dict[i][0] - self.pose.x)/abs(v_mag * (cos(self.pose.theta) - cos(self.all_agents_pose_dict[i][2])))


                    self.VX[i] = (np.asarray([self.point_to_agent_heading[i] - self._omega[i],self.point_to_agent_heading[i] + self._omega[i]]))
                    self.num = v_mag * sin(self.present_temp_h) + self.all_agents_pose_dict[i][3] * sin(self.all_agents_pose_dict[i][2])
                    self.den = v_mag * cos(self.present_temp_h) + self.all_agents_pose_dict[i][3] * cos(self.all_agents_pose_dict[i][2])
                    self.RVO[i] = (self.VX[i] + atan2(self.num,self.den))/2

                    if (self._distance < self._least_distance):
                        self._least_distance = self._distance

        self.vel_msg.linear.x = (self._least_distance/3)/self.NR
        #print(self.time_to_collision)

                    #print(self.agent_name)
                    #print("A2A heading:")
                    #print(self._rel_heading[i])
                    #print("Omega:")
                    #print(self._omega[i])

    # Returns True when called if the agent is on collision course
    def collision(self):
        if(self.in_RVO(self.pose.theta) == True):
            #if True, return True. Else, False.
            return True
        return False

    #Returns a new velocity that is outside VO
    def choose_new_velocity_VO(self):
        #Find the nearest heading that is outside the VO
        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        self._headings_array = np.round(np.arange(-np.pi,np.pi,0.01),2)

        # if not available, self.inside will return None.
        self.best_min = None

        #Find the nearest heading that is outside the VO
        self.temp_array_marginals = np.array([])
        print(self.VO)
        for i in self.VO:
            self.temp_array_marginals = np.append(self.temp_array_marginals, self.VO[i])
            #self.temp_temp_temp = self.VO[i][0]
        self._h = np.round(self.temp_array_marginals, 2)
        for i in range(len(self._h)):
            if(i%2==0):
                k = self._h[i]
                while(k <= np.round(self._h[i+1],2)):
                    self._headings_array = np.delete(self._headings_array, np.where(self._headings_array == np.round(k,2)))
                    k+=0.01
        print("===")
        print("RVO is :")
        print(self._h)
        self.idx = np.abs(self._headings_array - self.desired_heading -0.1).argmin()
        self.best_min = self._headings_array[self.idx]
        print("desired heading :")
        print(self.desired_heading)
        print("choosen direction is")
        print(self.best_min)
        print("===")
        #rospy.sleep(1)
        return self.best_min

    #Returns a new velocity that is outside RVO
    def choose_new_velocity_RVO(self):
        rr = 2
        incr = 0.01
        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        self._headings_array = np.round(np.arange(-np.pi,np.pi,incr),rr)

        # if not available, self.inside will return None.
        self.best_min = None

        #Find the nearest heading that is outside the VO
        self.temp_array_marginals = np.array([])
        #print(self.RVO)
        for i in self.RVO:
            self.temp_array_marginals = np.append(self.temp_array_marginals, self.RVO[i])
            #self.temp_temp_temp = self.RVO[i][0]
        self._h = np.round(self.temp_array_marginals, rr)

        #defining possible headings with a resolution of 0.01
        for i in range(len(self._h)):
            if(i%2==0):
                k = self._h[i] + incr
                while(k < np.round(self._h[i+1],rr)):
                    #if(len(self._h) >1):
                    self._headings_array = np.delete(self._headings_array, np.where(self._headings_array == np.round(k,rr)))
                    k+=incr
        #choosing heading nearest to goal heading
        #self._min_time_collision = self.time_to_collision(min(self.time_to_collision, key = self.time_to_collision.get))
        #self._min_time_collision = min(self.time_to_collision.items(), key=lambda x: x[1])
        self.idx = np.abs(self._headings_array - self.desired_heading).argmin()
        #self.idx = (np.abs(self._headings_array - self.desired_heading) + 0.01/(self._min_time_collision+0.0001)).argmin()
        # choose whether left or right side is the nearest and then assign
        self.best_min = self._headings_array[(self.idx-1)%len(self._headings_array)]
        print("RVO is :")
        print(self._h)
        print("===")
        print("desired heading :")
        print(self.desired_heading)
        print("choosen direction is")
        print(self.best_min)
        print("===")
        #rospy.sleep(1)
        return self.best_min
# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# RVO functions
    def move2goal_vo(self,x,y):
        self.goal_pose = Pose()

        # Get the input from the function call.
        self.goal_pose.x = x
        self.goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.2

        # Setting the velocity
        #self.vel_msg = Twist()
        #self.vel_msg.linear.x = 3
        #self.set_goal_heading(x,y)
        #self.velocity_publisher.publish(self.vel_msg)

        #rospy.sleep(10)
        while self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            self.vel_msg = Twist()
            self.vel_msg.linear.x = 0.1
            #self.velocity_publisher.publish(self.vel_msg)
            #rospy.sleep(0.5)
            if(self.collision() == True):
                #vel_msg.linear.x = np.min(rvo[0]-self.heading,rvo[-1]-self.heading)+self.heading
                #self.heading = self.rvo[np.where(np.min(abs(np.subtract(self.rvo,self.heading))))]
                print("Inside VO. Should choose new velocity")
                print("The new choosen velocity is : ")
                self.heading = self.choose_new_velocity_VO()
                #print(self.heading)
                #self.heading = self.VO[]
                self.set_heading(self.heading)
                self.vel_msg.linear.x = 0.1
            else:
                self.set_goal_heading(x,y)

            self.velocity_publisher.publish(self.vel_msg)
            self.publish_to_information_channel(self.agent_name)
            #print(self.all_agents_pose_dict)
            #   rospy.sleep(0.1)

        # Stopping the agent after the movement is over.
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()

    def move2goal_rvo(self,x,y):
        self.goal_pose = Pose()

        # Get the input from the function call.
        self.goal_pose.x = x
        self.goal_pose.y = y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.2

        # Setting the direction and velocity
        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        self.set_heading(self.desired_heading)

        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5
        self.velocity_publisher.publish(self.vel_msg)

        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        self.heading = self.desired_heading

        while self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            #self.vel_msg.linear.x = 0.5
            self.update_RVO(self.vel_msg.linear.x)

            if(self.collision() == True):
                print("1")
                #print("Inside RVO. Should choose new velocity")
                #print("The new choosen velocity is : ")
                #self.heading = self.choose_new_velocity_VO()
                self.heading = self.choose_new_velocity_RVO()
                if (self.best_min == None):
                    #self.vel_msg.linear.x = self.penalize(self.vel_msg.linear.x)
                    print("#########################################")
                    self.heading = self.prev_heading
                    #self.vel_msg.linear.x = 0.1
                self.set_heading(self.heading)
                #print(self.heading)
                #print("---")
                #self.heading = self.VO[]
                #self.set_heading(self.heading)
                #rospy.sleep(0.01)
            else:
                self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
                if(self.in_RVO(self.desired_heading) == True):
                    print("2")
                    #print("desired heading still inside. Continue prev heading")
                    #self.vel_msg.linear.x = 0
                    #self.heading = self.prev_heading
                    self.heading = self.choose_new_velocity_RVO()
                else:
                    print("3")
                    self.heading = self.desired_heading
                self.set_heading(self.heading)

            self.velocity_publisher.publish(self.vel_msg)
            self.publish_to_information_channel(self.agent_name)
            self.prev_heading = self.heading
            print("-----")

        # Stopping the agent after the movement is over.
        self.vel_msg.linear.x = 0
        self.velocity_publisher.publish(self.vel_msg)

        # If we press control + C, the node will stop.
        #rospy.spin()


# end
#-----------------------------------------------------------------------------------------#

#-----------------------------------------------------------------------------------------#
# Testing functions
    def wander(self,lin_vel,ang_vel):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 1

        while 1:
            goal_pose.x = int(randint(0, 9))
            goal_pose.y = int(randint(0, 9))

            vel_msg = Twist()

            # Linear velocity in the x-axis.
            vel_msg.linear.x = lin_vel
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = ang_vel

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
# end
#-----------------------------------------------------------------------------------------#
