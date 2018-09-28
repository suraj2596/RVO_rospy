#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

import turtle_instance


if __name__ == '__main__':
    try:
        x = turtle_instance.TurtleBot("turtle1")   
        x.move2goal(9,5)
    except rospy.ROSInterruptException:
        pass
