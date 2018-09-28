#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

import turtle_instance

"""
import subprocess

spawn_turtle = 'rosservice call /spawn 2 2 0 "turtle2"'
subprocess.call(spawn_turtle)
"""

node_name = "turtle2"

if __name__ == '__main__':
    try:
        x = turtle_instance.TurtleBot(node_name)
        #x.move2goal(1,1)

        #wandering
        x.wander()
        while not rospy.is_shutdown():
            x.publish_to_information_channel(node_name)
            rospy.sleep(1)  # sleep for one second


    except rospy.ROSInterruptException:
        pass
