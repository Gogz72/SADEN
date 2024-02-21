#!/usr/bin/env python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy import interpolate
import matplotlib.pyplot as plt

def set_current_pose(data):
    global current_point, current_orientation

    current_point = ([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    current_orientation = ([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
   
    


def set_goal_pose(data):
    global goal_point, goal_orientation

    goal_point = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

    goal_orientation = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])



class Dot:
    def __init__(self, start, goal, obstacles, rep_radii, k_att=0.5, k_rep=1, max_iter=1000, threshold=0.1):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = np.array(obstacles)
        self.rep_radii = rep_radii
        self.k_att = k_att
        self.k_rep = k_rep
        self.max_iter = max_iter
        self.threshold = threshold
        self.positions = [self.start.copy()]  # Initialize list to store positions

    def attractive_potential(self, position):
        return 0.5 * self.k_att * np.linalg.norm(position - self.goal)**2

    def repulsive_potential(self, position):
        rep_potential = 0
        for i, obstacle in enumerate(self.obstacles):
            distance = np.linalg.norm(position - obstacle)
            if distance < self.rep_radii[i]:
                rep_potential += 0.5 * self.k_rep * (1 / distance - 1 / self.rep_radii[i])**2
        return rep_potential

    def total_potential(self, position):
        return self.attractive_potential(position) + self.repulsive_potential(position)

    def gradient(self, position):
        delta = 0.001
        dx = (self.total_potential(position + np.array([delta, 0, 0])) - self.total_potential(position - np.array([delta, 0, 0]))) / (2 * delta)
        dy = (self.total_potential(position + np.array([0, delta, 0])) - self.total_potential(position - np.array([0, delta, 0]))) / (2 * delta)
        dz = (self.total_potential(position + np.array([0, 0, delta])) - self.total_potential(position - np.array([0, 0, delta]))) / (2 * delta)
        return np.array([dx, dy, dz])

    def move(self):
        step_size = 0.1
        max_iter = self.max_iter
        threshold = self.threshold

        for _ in range(max_iter):
            gradient = self.gradient(self.start)

            # Update position based on gradient descent
            self.start -= step_size * gradient
            
                    # Create a PoseStamped message
            control_msg = PoseStamped()
            control_msg.header.stamp = rospy.Time.now()
            control_msg.pose.position.x = self.start[0]
            control_msg.pose.position.y = self.start[1]
            control_msg.pose.position.z = 1.0  # Assuming z-coordinate is 0 for control point
            control_msg.pose.orientation = Quaternion()  # Set orientation to identity

            # Publish the PoseStamped message
            control_pub.publish(control_msg)
            
            # Append the new position to the list of positions
            #self.positions.append(self.start.copy())




 
if __name__ == '__main__':

        rospy.init_node("apf")

        sub_current_pose = rospy.Subscriber('/Pos', PoseStamped, set_current_pose)
        sub_goal_pose = rospy.Subscriber('/Target_Pos', PoseStamped, set_goal_pose)
        control_pub = rospy.Publisher('/Control_Pose',PoseStamped, queue_size=10)

        current_point = np.array([0.0, 0.0, 0.0])  # Initialize as numpy array
        current_orientation = np.array([0.0, 0.0, 0.0])
        goal_point = np.array([0.0, 0.0, 0.0])  # Initialize as numpy array
        goal_orientation = np.array ([0.0, 0.0, 0.0])

        obstacles = np.array([[1, 2, 1], [4, 1, 1]])  # Convert to numpy array for easier calculations
        rep_radii = [1, 1.5]  # Radius for each obstacle

        #rate = rospy.rate(60)

       # my_dot = Dot(current_point, goal_point, obstacles, rep_radii)
        
        while not rospy.is_shutdown():
            
            my_dot = Dot(current_point, goal_point, obstacles, rep_radii)
            my_dot.move()

            #rate.sleep

        #my_dot.plot_movement()
