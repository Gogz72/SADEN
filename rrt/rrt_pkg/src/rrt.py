#!/usr/bin/python3
import cv2
import numpy as np
import math
import random
import argparse
import os
import rospy 


from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header

class RosRRT:
    def __init__(self):
        rospy.init_node('rrt_path_planner', anonymous=True)
        self.target_pose_subscriber = rospy.Subscriber('/Target_Pos', PoseStamped, self.target_pose_callback)
        self.path_publisher = rospy.Publisher('/rrt_path', Path, queue_size=10)
        self.test_publisher = rospy.Publisher('/test', PoseStamped, queue_size=10)
        self.target_pose = None

    def target_pose_callback(self, msg):
        # Extracting the pose (x, y, z) from PoseStamped message
        self.target_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        print(self.target_pose)
        self.test_publisher.publish(self.target_pose)

    def publish_path(self, node_list):
        path = Path()
        path.header = Header()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "world"  # Adjust frame_id as needed

        for node in node_list:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose.position.x = node.x
            pose_stamped.pose.position.y = node.y
            pose_stamped.pose.position.z = 0  # Assuming 2D path planning; set z to 0 or modify as needed
            path.poses.append(pose_stamped)

        self.path_publisher.publish(path)

    def run_rrt(self, img, img2, start, max_nodes, stepSize):
        while not rospy.is_shutdown():
            if self.target_pose is not None:
                # Assuming 2D path planning; using x and y only
                target_2d = (self.target_pose[0], self.target_pose[1])
                RRT(img, img2, start, target_2d, max_nodes, stepSize, self.publish_path)

# Rest of the code including the RRT function and main block remains the same...


class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

def collision(x1, y1, x2, y2, img):
    color = []
    x = list(np.arange(x1, x2, (x2 - x1) / 100))
    y = list(((y2 - y1) / (x2 - x1)) * (x - x1) + y1)
    for i in range(len(x)):
        color.append(img[int(y[i]), int(x[i])])
    return 0 in color  # True if collision, False otherwise

def check_collision(x1, y1, x2, y2, img, stepSize):
    _, theta = dist_and_angle(x2, y2, x1, y1)
    x = x2 + stepSize * np.cos(theta)
    y = y2 + stepSize * np.sin(theta)

    hy, hx = img.shape
    if y < 0 or y > hy or x < 0 or x > hx:
        return x, y, False

    nodeCon = not collision(x, y, x2, y2, img)
    return x, y, nodeCon

def dist_and_angle(x1, y1, x2, y2):
    dist = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    angle = math.atan2(y2 - y1, x2 - x1)
    return dist, angle

def nearest_node(x, y, node_list):
    temp_dist = []
    for i in range(len(node_list)):
        dist, _ = dist_and_angle(x, y, node_list[i].x, node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

def rnd_point(h, l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return new_x, new_y

def RRT(img, img2, start, max_nodes, stepSize):
    h, l = img.shape
    node_list = [Nodes(start[0], start[1])]
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    cv2.circle(img2, start, 5, (0, 0, 255), thickness=3, lineType=8)

    node_count = 1
    while node_count < max_nodes:
        nx, ny = rnd_point(h, l)
        nearest_ind = nearest_node(nx, ny, node_list)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y

        tx, ty, nodeCon = check_collision(nx, ny, nearest_x, nearest_y, img, stepSize)

        if nodeCon:
            node_list.append(Nodes(tx, ty))
            node_list[node_count].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[node_count].parent_y = node_list[nearest_ind].parent_y.copy()
            node_list[node_count].parent_x.append(tx)
            node_list[node_count].parent_y.append(ty)

            cv2.circle(img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(img2, (int(tx), int(ty)), (int(node_list[nearest_ind].x), int(node_list[nearest_ind].y)), (0, 255, 0), thickness=1, lineType=8)
            cv2.imwrite("media/" + str(node_count) + ".jpg", img2)
            cv2.imshow("sdc", img2)
            cv2.waitKey(1)

            node_count += 1

    cv2.imwrite("final_map_exploration.jpg", img2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Below are the params:')
    parser.add_argument('-p', type=str, default="/home/panda/Desktop/SADEM/new_slam/catkin_ws/src/rrt_pkg/src/world1.png", metavar='ImagePath', action='store', dest='imagePath', help='Path of the image containing mazes')
    parser.add_argument('-s', type=int, default=10, metavar='Stepsize', action='store', dest='stepSize', help='Step-size to be used for RRT branches')
    parser.add_argument('-start', type=int, default=[20, 20], metavar='startCoord', dest='start', nargs='+', help='Starting position in the maze')
    parser.add_argument('-max_nodes', type=int, default=1000, metavar='MaxNodes', action='store', dest='max_nodes', help='Maximum number of nodes for exploration')
    parser.add_argument('-selectPoint', help='Select start point from figure', action='store_true')

    args = parser.parse_args()

    try:
        os.system("rm -rf media")
    except:
        print("Dir already clean")
    os.mkdir("media")

    img = cv2.imread(args.imagePath, 0)
    img2 = cv2.imread(args.imagePath)
    start = tuple(args.start)
    stepSize = args.stepSize
    max_nodes = args.max_nodes

    homos = RosRRT

    coordinates = []
    if args.selectPoint:
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', draw_circle)
        while True:
            cv2.imshow('image', img2)
            if cv2.waitKey(20) & 0xFF == 27:
                break
        start = (coordinates[0], coordinates[1])

    RRT(img, img2, start, max_nodes, stepSize)
    cv2.destroyAllWindows()

