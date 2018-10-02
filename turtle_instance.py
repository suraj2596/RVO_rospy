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


        self.publish_information.publish(i)
        #print("Published the turtle node name on topic /common_information")
        self.rate.sleep()

    def recieve_from_information_channel(self,data):
        self.inf = data
        self.name_temp = self.inf.agent_name
        self.x_temp = self.inf.agent_pose_x
        self.y_temp = self.inf.agent_pose_y
        self.heading_temp = self.inf.agent_heading

        #if(self.name_temp == self.agent_name):
        self.pose_updated = [self.x_temp, self.y_temp, self.heading_temp]
        #all_agents_pose_dict[self.name_temp] = 0
        self.all_agents_pose_dict.update({self.name_temp: self.pose_updated})

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

    def start_point(self,x,y):
        rospy.sleep(0.1)
        self.turtle_teleport(x,y,0)

    def in_VO(self,h):
        #print(len(self.VO))
        for i in self.VO:
            #print(h)
            #print(self.RVO[i])
            if(self.VO[i][0] < h[i] < self.VO[i][1]):
                return True
                break
        return False

    def in_RVO(self,h):
        #print(len(self.VO))
        for i in self.RVO:
            #print(h)
            #print(self.RVO[i])
            if(self.RVO[i][0] < h[i] < self.RVO[i][1]):
                return True
                break
        return False

    def inside2(self,s,h):
        #print(len(self.VO))
        #print(h)
        self.best_min_temp = None
        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        #print(len(h))
        while(len(h)!=0):
            self.idx = np.abs(np.asarray(h) - 0.1- self.desired_heading).argmin()
            print(h)
            for j in s:
                #print(j)
                #print(s[j])
                if(s[j][0] < h[self.idx] < s[j][1]):
                    print("The value of h is %f." %h[self.idx])
                    h = np.delete(h,self.idx)
                    self.inside(s,h)
                else:
                    continue
            self.best_min_temp = h[self.idx]
            break
        return self.best_min_temp

    # Returns True when called if the agent is on collision course
    def collision(self,v_mag,turtle_name=0,r=5):
        #calc the relative velocity of the agent and choosen other agent
        self._rel_heading = {}
        self.point_to_agent_heading = {}
        self._omega = {}
        self.VO = {}
        self.RVO = {}
        rospy.sleep(0.01)
        #print(self.all_agents_pose_dict)
        self.present_temp_h = round(self.pose.theta,1)
        for i in self.all_agents_pose_dict:
            print(i)
            if(i != self.agent_name):
                #calc the relative velocity of the agent and choosen other agent
                self._rel_v_x =  v_mag * (cos(self.pose.theta) - cos(self.all_agents_pose_dict[i][2]))
                self._rel_v_y =  v_mag * (sin(self.pose.theta) - sin(self.all_agents_pose_dict[i][2]))
                self._rel_heading[i] = round(atan2(self._rel_v_y,self._rel_v_x),1)
                #print(self._rel_v_x)
                #print(self._rel_heading[i])
                #check if this heading/velocity is in the VO that is
                #updated by VO finder in "recieve_from_information_channel"

                # VO finder :: Should output a range of headings into an 2D array
                self.point_to_agent_heading[i] = round(atan2((self.all_agents_pose_dict[i][1] - self.pose.y),(self.all_agents_pose_dict[i][0] - self.pose.x)),1)
                #print(self.point_to_agent_heading[i])

                self._distance = round(sqrt(pow((self.all_agents_pose_dict[i][0] - self.pose.x), 2) + pow((self.all_agents_pose_dict[i][1] - self.pose.y), 2)),1)
                try:
                    self._omega[i] = round(asin(r/self._distance),1)
                except ValueError:
                    self._omega[i] = round(np.pi/2,1)

                self.VO[i] = ([self.point_to_agent_heading[i] - self._omega[i],self.point_to_agent_heading[i] + self._omega[i]])
                self.RVO[i] = np.add(np.asarray(self.VO[i]),self.present_temp_h)/2
                #self.RVO[i] = np.add(np.asarray(self.VO[i]),self._rel_heading[i])/2
                print("===")
                print("A2A heading:")
                print(self._rel_heading[i])
                print("Omega:")
                print(self._omega[i])

        if(self.in_RVO(self._rel_heading) == True):
            #if True, return True. Else, False.
            return True
            print("True")
        else:
            return False
            print("False")

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

    def choose_new_velocity_RVO(self):
        self.desired_heading = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)
        self._headings_array = np.round(np.arange(-np.pi,np.pi,0.005),3)

        # if not available, self.inside will return None.
        self.best_min = None

        #Find the nearest heading that is outside the VO
        self.temp_array_marginals = np.array([])
        print(len(self.RVO))
        for i in self.RVO:
            self.temp_array_marginals = np.append(self.temp_array_marginals, self.RVO[i])
            #self.temp_temp_temp = self.RVO[i][0]
        self._h = np.round(self.temp_array_marginals, 1)
        for i in range(len(self._h)):
            if(i%2==0):
                k = self._h[i] + 0.1
                while(k < np.round(self._h[i+1],1)):
                    self._headings_array = np.delete(self._headings_array, np.where(self._headings_array == np.round(k,1)))
                    k+=0.1
        print("RVO is :")
        print(self._h)
        self.idx = np.abs(self._headings_array - self.desired_heading -0.2).argmin()
        self.best_min = self._headings_array[self.idx]
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
            if(self.collision(self.vel_msg.linear.x) == True):
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

        # Setting the velocity
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.5
        #self.set_goal_heading(x,y)
        self.velocity_publisher.publish(self.vel_msg)
        #   os.system('clear')
        #rospy.sleep(0.5)
        while self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            if(self.collision(self.vel_msg.linear.x) == True):
                print("Inside RVO. Should choose new velocity")
                #print("The new choosen velocity is : ")
                #self.heading = self.choose_new_velocity_VO()
                self.heading = self.choose_new_velocity_RVO()
                if (self.best_min == None):
                    #self.vel_msg.linear.x = self.penalize(self.vel_msg.linear.x)
                    print("#########################################")
                    self.heading = self.prev_heading
                    self.vel_msg.linear.x = 0.1
                else:
                    self.vel_msg.linear.x = 0.5
                #print(self.heading)
                print("---")
                #self.heading = self.VO[]
                self.set_heading(self.heading)
            else:
                print("Outside.")
                #rospy.sleep(0.1)
                self.set_goal_heading(x,y)


            self.prev_heading = self.heading
            self.velocity_publisher.publish(self.vel_msg)
            self.publish_to_information_channel(self.agent_name)
            #print(self.all_agents_pose_dict)
            #   rospy.sleep(0.1)

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


"""if __name__ == '__main__':
    try:
        x1 = TurtleBot("turtle1")
        x1.move2goal_rvo(1,8)

    except rospy.ROSInterruptException:
        pass
"""
