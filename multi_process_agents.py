#!/usr/bin/env python

import multiprocessing

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

import numpy as np

from turtle_instance import TurtleBot

k = 0
sp_x = [1,9,1,9]
sp_y = [1,9,9,1]

goal_x = [9,1,9,1]
goal_y = [9,1,1,9]

def multi_agents(agent_name,agent_obj,sp_x,sp_y,goal_x,goal_y,p_name):
    try:
        agent_obj = TurtleBot(agent_name)
        agent_obj.start_point(sp_x,sp_y)
        agent_obj.move2goal_vo(goal_x,goal_y)

    except rospy.ROSInterruptException:
        pass

user_input = int(input("Type no. of agents : ")) +1
agent_names, agent_obj,p_name = [None] * (user_input), [None] * (user_input), [None] * (user_input)
sp_x,sp_y,goal_x,goal_y = [None] * (user_input),[None] * (user_input),[None] * (user_input),[None] * (user_input)

#Equal distribution for start_point
r = 5
c = [5.5,5.5]
_angle = 2*(np.pi/user_input)
_pad_angle_sp = 0
_pad_angle_goal = 0

#define turtle 0 outside loop
for i in range(user_input):
    agent_names[i] = "turtle" + str(i)
    agent_obj[i] = "x" + str(i)
    sp_x[i] = c[0] + r*np.cos(_angle*i + _pad_angle_sp)
    sp_y[i] = c[1] + r*np.sin(_angle*i + _pad_angle_sp)
    goal_x[i] = c[0] - r*np.cos(_angle*i + _pad_angle_goal)
    goal_y[i] = c[1] - r*np.sin(_angle*i + _pad_angle_goal)

#print(agent_names)

for i in agent_names:
    p_name[k] = "p"+str(k)
    p_name[k] = multiprocessing.Process(target=multi_agents, args=(agent_names[k], agent_obj[k], sp_x[k], sp_y[k], goal_x[k], goal_y[k], p_name, ))
    k += 1

for i in p_name:
    i.start()
