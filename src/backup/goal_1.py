#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

import turtle_instance

node_name = "turtle1"

if __name__ == '__main__':
    try:
        x = turtle_instance.TurtleBot(node_name)

        #test
        """x.move2goal(9,5)
        x.set_heading(-45)
        x.move2goal(9,2)
        x.set_heading(-180)
"""
        #wandering
        #x.wander()
        x.move2goal_rvo(9,9)
        """while not rospy.is_shutdown():
            x.publish_to_information_channel(node_name)
            rospy.sleep(1)  # sleep for one second"""


    except rospy.ROSInterruptException:
        pass
