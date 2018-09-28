#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

from turtle_instance import TurtleBot

agent_name = "turtle4"

if __name__ == '__main__':
    try:
        x4 = TurtleBot(agent_name)

        #test
        """x.move2goal(9,5)
        x.set_heading(-45)
        x.move2goal(9,2)
        x.set_heading(-180)
"""
        #wandering
        #x2.wander(1,3)
        #x2.start_point(9,9)
        #x2.move2goal_vo(1,1)
        x4.start_point(1,9)
        x4.move2goal_rvo(1,9)
        #x2.move2goal_rvo(9,1)
        #x2.move2goal_rvo(1,1)

    except rospy.ROSInterruptException:
        pass
