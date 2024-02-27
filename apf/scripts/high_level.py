#!/usr/bin/env python3

import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import random

def set_current_pose(data):
    global current_point, current_orientation

    current_point = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    current_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])


def obstacle_coordinates(drone_x, drone_y, drone_orientation, distance):
    # Convert sensor direction to angle relative to world frame
    angle = drone_orientation 
    # Convert angle to radians
    # Calculate obstacle coordinates relative to world frame
    obstacle_x = drone_x + distance * np.cos(angle)
    obstacle_y = drone_y + distance * np.sin(angle)
    return obstacle_x, obstacle_y

def set_obstacle_distance (data):
    global obstacle_length
    obstacle_length = data
    
    
   
def set_goal_pose(data):
    global goal_point, goal_orientation

    goal_point = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    goal_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

def pi_2_pi(angle):
    #A function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + np.pi) % (2 * np.pi) - np.pi

def trig (x,y,x_c,y_c):
    angle_rad = math.atan2(y - y_c, x - x_c)
    #angle_deg = math.degrees(angle_rad)
    if angle_rad < 0:
        angle_rad += np.pi*2
    return angle_rad


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
    obstacle_pub = rospy.Publisher('/obstacle_coord',PoseStamped,queue_size=10)
    obstacle_sub = rospy.Subscriber('/obstacle',Float32,set_obstacle_distance)

    current_point = np.array([0.0, 0.0, 0.0])  # Initialize as numpy array
    obstacle_length = -1.0
    obstacle_coords = PoseStamped()


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

            obstacle_coords.pose.position.x, obstacle_coords.pose.position.y = obstacle_coordinates(current_point[0],current_point[1],z_th,obstacle_length)
            obstacle_pub.publish()
            set_point(x, y, z, np.pi/1000, np.pi/1000, z_th)   

        x_c = x
        y_c = y
    






