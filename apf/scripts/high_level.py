#!/usr/bin/env python3

import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import random

def set_current_pose(data):
    global current_point, current_orientation

    current_point = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    current_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
   
def set_goal_pose(data):
    global goal_point, goal_orientation

    goal_point = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    goal_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

def pi_2_pi(angle):
    #A function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + np.pi) % (2 * np.pi) - np.pi

def trig (x,y,x_c,y_c):
    pythegorous = np.sqrt((x_c - x)**2 + (y_c - y)**2)
    angle = np.arcsin((y_c - y)/pythegorous)
    return -angle


def clock_wise (angle):
    angle = angle + (np.pi/2)
    angle = pi_2_pi(angle)
    return angle

def anti_clock_wise (angle):
    angle = angle - (np.pi/2)
    angle = pi_2_pi(angle)
    return angle

def one_eighty (angle):
    angle = angle + np.pi
    angle = pi_2_pi(angle)
    return angle

def set_point(x,y,z,x_th,y_th,z_th):

    target_msg = PoseStamped()
    target_msg.header.stamp = rospy.Time.now()
    target_msg.pose.position.x = x
    target_msg.pose.position.y = y
    target_msg.pose.position.z = z  # Assuming z-coordinate is 0 for control point
    target_msg.pose.orientation = Quaternion(*quaternion_from_euler(x_th, y_th, z_th))

    target_pub.publish(target_msg)

if __name__ == '__main__':
    rospy.init_node("high_level")

    sub_current_pose = rospy.Subscriber('/Pos', PoseStamped, set_current_pose)
    sub_goal_pose = rospy.Subscriber('/Target_Pos', PoseStamped, set_goal_pose)
    target_pub = rospy.Publisher('/Target_commands',PoseStamped, queue_size=10)

    current_point = np.array([0.0, 0.0, 0.0])  # Initialize as numpy array
    current_orientation = np.array([0.0, 0.0, 0.0, 0.0])
    goal_point = np.array([0.0, 0.0, 0.0])  # Initialize as numpy array
    goal_orientation = np.array([0.0, 0.0, 0.0, 0.0])

    x = 0.0
    y = 0.0
    z = 1.0
    x_c = 0.0
    y_c = 0.0


    step_size = 1.0

    step_size_time = 10.0

    #time.sleep(step_size_time)


    while not rospy.is_shutdown():

        x = float(random.randint(-5, 5)) 
        y = float(random.randint(-5, 5))

        z_th = trig(x,y,x_c,y_c)
        
        start_time = time.time()

        while time.time() - start_time < 10:
            # Your code here

            set_point(x_c,y_c,z,np.pi/1000,np.pi/1000,z_th)
            
        start2_time = time.time()
        while time.time() - start2_time < 20:
            # Your code here
            
            set_point(x, y, z, np.pi/1000, np.pi/1000, z_th)   

        x_c = x
        y_c = y
    






