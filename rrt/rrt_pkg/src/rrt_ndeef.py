#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import random
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header

class RRTPlanner3D:
    def __init__(self):
        rospy.init_node('rrt_planner_3d', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/Pos', PoseStamped, self.pose_callback)
        self.target_subscriber = rospy.Subscriber('/Target_Pos', PoseStamped, self.target_callback)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        self.target_pose = None
        self.robot_pose = None
        self.path = None

    def pose_callback(self, msg):
        self.robot_pose = msg

    def target_callback(self, msg):
        self.target_pose = msg

    def rrt(self, max_iterations=1000):
        if self.robot_pose is None or self.target_pose is None:
            return

        start = (
            self.robot_pose.pose.position.x,
            self.robot_pose.pose.position.y,
            self.robot_pose.pose.position.z
        )
        goal = (
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z
        )

        # Initialize RRT
        tree = {start: None}
        path_found = False

        for _ in range(max_iterations):
            random_point = (
                random.uniform(0, 10),
                random.uniform(0, 10),
                random.uniform(0, 10)
            )  # Adjust bounds as needed
            nearest_node = self.find_nearest_node(tree, random_point)
            new_node = self.steer(nearest_node, random_point)

            tree[new_node] = nearest_node

            if self.distance(new_node, goal) < 0.1:
                tree[goal] = new_node  # Add the goal to the tree
                path_found = True

        if path_found:
            self.path = self.extract_path(tree, start, goal)
            self.publish_path()

    def find_nearest_node(self, tree, point):
        nearest_node = None
        min_distance = float('inf')

        for node in tree.keys():
            distance = self.distance(node, point)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node

    def steer(self, from_node, to_point, step_size=0.1):
        distance = self.distance(from_node, to_point)

        if distance < step_size:
            return to_point

        theta = math.atan2(to_point[1] - from_node[1], to_point[0] - from_node[0])
        phi = math.atan2(to_point[2] - from_node[2], math.sqrt((to_point[0] - from_node[0])**2 + (to_point[1] - from_node[1])**2))
        new_x = from_node[0] + step_size * math.cos(theta) * math.sin(phi)
        new_y = from_node[1] + step_size * math.sin(theta) * math.sin(phi)
        new_z = from_node[2] + step_size * math.cos(phi)
        return (new_x, new_y, new_z)

    def distance(self, point1, point2):
        return math.sqrt(
            (point1[0] - point2[0]) ** 2 +
            (point1[1] - point2[1]) ** 2 +
            (point1[2] - point2[2]) ** 2
        )

    def extract_path(self, tree, start, goal):
        if goal not in tree:
            return None  # No path found

        path = [goal]
        current_node = goal

        while current_node != start:
            parent_node = tree[current_node]
            path.append(parent_node)
            current_node = parent_node

        path.reverse()
        return path

    def publish_path(self):
        if self.path is not None:
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "world"  # Adjust frame_id as needed

            for point in self.path:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = point[0]
                pose_stamped.pose.position.y = point[1]
                pose_stamped.pose.position.z = point[2]
                path_msg.poses.append(pose_stamped)

            self.path_publisher.publish(path_msg)
            print(path_msg)

if __name__ == '__main__':
    planner = RRTPlanner3D()

    while not rospy.is_shutdown():
        planner.rrt()
