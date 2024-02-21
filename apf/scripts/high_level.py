import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def clock_wise (angle):
    angle = angle + (np.pi/2)
    return angle

def anti_clock_wise (angle):
    angle = angle - (np.pi/2)
    return angle

def one_eighty (angle):
    angle = angle + np.pi
    return angle

