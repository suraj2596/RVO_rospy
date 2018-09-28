#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

from turtle_instance import TurtleBot

agent_name_1 = "turtle1"

if __name__ == '__main__':
    try:
        x1 = TurtleBot(agent_name_1)
        #x2 = TurtleBot(agent_name_2)

        """x1(agent_name_1)
        x2(agent_name_2)
"""
        #test
        """x.move2goal(9,5)
        x.set_heading(-45)
        x.move2goal(9,2)
        x.set_heading(-180)
"""
        #wandering
        #x1.wander(3,2)
        #x1.start_point(1,1)
        #x1.move2goal_vo(9,9)
        #rospy.sleep(0.5)
        x1.start_point(1,1)
        x1.move2goal_rvo(9,9)

        #x1.move2goal_rvo(3,7)
        #x1.move2goal_rvo(9,1)
        #x1.move2goal_rvo(1,9)

        #x2.move2goal_rvo(1,8)


    except rospy.ROSInterruptException:
        pass
