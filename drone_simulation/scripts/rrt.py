#!/usr/bin/env python3
import random
import math
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path

# Node class for RRT*
class Node:
    def __init__(self, point):
        self.point = point
        self.cost = 0.0
        self.parent = None

# Distance calculation
def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

# Steer function for RRT*
def steer(from_node, to_point, extend_length=float('inf')):
    if distance(from_node.point, to_point) < extend_length:
        return Node(to_point)
    else:
        new_point = [
            from_node.point[0] + (to_point[0] - from_node.point[0]) * extend_length / distance(from_node.point, to_point),
            from_node.point[1] + (to_point[1] - from_node.point[1]) * extend_length / distance(from_node.point, to_point),
            from_node.point[2] + (to_point[2] - from_node.point[2]) * extend_length / distance(from_node.point, to_point)
        ]
        return Node(new_point)

# Nearest node function for RRT*
def nearest_node(tree, point):
    nearest = tree[0]
    for node in tree:
        if distance(node.point, point) < distance(nearest.point, point):
            nearest = node
    return nearest

# Choose parent for the new node
def choose_parent(new_node, nearest, tree, radius, Map):
    nodes = []
    for node in tree:
        if distance(node.point, new_node.point) <= radius and collision_free(node, new_node, Map):
            nodes.append(node)
    
    if not nodes:
        return nearest

    min_cost = nearest.cost + distance(nearest.point, new_node.point)
    for node in nodes:
        cost = node.cost + distance(node.point, new_node.point)
        if cost < min_cost:
            min_cost = cost
            new_node.parent = node

    new_node.cost = min_cost
    return new_node.parent

# Rewire the tree
def rewire(tree, new_node, radius, Map):
    for node in tree:
        if node != new_node.parent and distance(node.point, new_node.point) <= radius and new_node.cost + distance(new_node.point, node.point) < node.cost:
            if collision_free(new_node, node, Map):
                node.parent = new_node
                node.cost = new_node.cost + distance(new_node.point, node.point)

# Collision checking
def collision_free(node1, node2, Map):
    return not is_line_in_collision(node1.point, node2.point, Map)

# Bresenham's Line Algorithm in 3D for collision checking
def is_line_in_collision(start, end, Map):
    points = get_line_points(start, end)
    for point in points:
        if Map[int(point[0]), int(point[1]), int(point[2])] > 0:
            return True
    return False

# Getting points in a 3D line
def get_line_points(start, end):
    points = []
    x1, y1, z1 = start
    x2, y2, z2 = end
    dx, dy, dz = abs(x2 - x1), abs(y2 - y1), abs(z2 - z1)
    xs = 1 if x2 > x1 else -1
    ys = 1 if y2 > y1 else -1
    zs = 1 if z2 > z1 else -1

    # Driving axis is X-axis
    if dx >= dy and dx >= dz:        
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while x1 != x2:
            x1 += xs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dx
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            points.append((x1, y1, z1))

    # Driving axis is Y-axis
    elif dy >= dx and dy >= dz:        
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while y1 != y2:
            y1 += ys
            if p1 >= 0:
                x1 += xs
                p1 -= 2 * dy
            if p2 >= 0:
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            points.append((x1, y1, z1))

    # Driving axis is Z-axis
    else:        
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while z1 != z2:
            z1 += zs
            if p1 >= 0:
                y1 += ys
                p1 -= 2 * dz
            if p2 >= 0:
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            points.append((x1, y1, z1))

    return points

# Generate final path
def generate_path(goal_node):
    path = []
    node = goal_node
    while node.parent is not None:
        path.append(node.point)
        node = node.parent
    path.append(node.point)
    return path[::-1]

# Convert path to ROS message
def convert_path_to_ros_msg(path):
    path_msg = Path()
    path_msg.header.frame_id = "map"  # Adjust the frame as per your setup
    path_msg.header.stamp = rospy.Time.now()

    for point in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.w = 1.0  # Assuming no orientation change
        path_msg.poses.append(pose)

    return path_msg

# RRT* algorithm
def rrt_star(start_point, goal_point, iterations, map_dimensions, Map, radius=1.5, extend_length=1.0):
    tree = [Node(start_point)]

    for _ in range(iterations):
        random_point = [random.uniform(map_dimensions[0], map_dimensions[1]), 
                        random.uniform(map_dimensions[2], map_dimensions[3]),
                        random.uniform(map_dimensions[4], map_dimensions[5])]

        nearest = nearest_node(tree, random_point)
        new_node = steer(nearest, random_point, extend_length)

        if collision_free(nearest, new_node, Map):
            new_node.parent = choose_parent(new_node, nearest, tree, radius, Map)
            tree.append(new_node)
            rewire(tree, new_node, radius, Map)

            if distance(new_node.point, goal_point) <= extend_length and collision_free(new_node, Node(goal_point), Map):
                return generate_path(new_node)

    return None # No path found

# ROS and CoppeliaSim Integration
if __name__ == '__main__':
    rospy.init_node('Path_planning', anonymous=True)

    # Initialize ROS Publishers, Subscribers, and other necessary variables
    path_publisher = rospy.Publisher('/path', Path, queue_size=10)

    # Set up your Map here (3D numpy array representing the environment)
    Map = np.zeros((100, 100, 100))  # Replace with actual map data

    # RRT* Path Planning
    start_point = [0, 0, 0]  # Replace with actual start point
    goal_point = [10, 10, 10]  # Replace with actual goal point
    path = rrt_star(start_point, goal_point, iterations=1000, map_dimensions=[0, 100, 0, 100, 0, 100], Map=Map)

    # Convert and Publish Path for ROS
    if path:
        path_msg = convert_path_to_ros_msg(path)
        path_publisher.publish(path_msg)
        rospy.loginfo("Path published")
    else:
        rospy.loginfo("No path found")

    rospy.spin()

