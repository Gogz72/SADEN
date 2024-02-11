#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import rospy
#from std_msgs.msg import Float32MultiArray,Float32
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point

def set_current_pose(data):
	global current_point,current_posestamped,start_point2
	current_posestamped=data
	start_point2=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
	current_point=convert_copelia_to_map([data.pose.position.x,data.pose.position.y,data.pose.position.z])


def set_goal_pose(data):
	global goal_point,goal_point2,goal_point_prev,goal_changed,counter,goal_posestamped
	goal_posestamped=data
	goal_point2=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
	goal_point=convert_copelia_to_map([data.pose.position.x,data.pose.position.y,data.pose.position.z])
	if goal_point!=goal_point_prev:
		#print("Goal Pose changed")
		goal_changed=True
		counter=0
	else:
		if counter>=10 and goal_changed==True:
			sub_goal_pose.unregister()
		counter+=1
		#print("Goal Pose same",counter)
		#goal_changed=False
	goal_point_prev=goal_point
	
	#print("Goal Pose Set",goal_pose)


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
        dx = (self.total_potential(position + np.array([delta, 0])) - self.total_potential(position - np.array([delta, 0]))) / (2 * delta)
        dy = (self.total_potential(position + np.array([0, delta])) - self.total_potential(position - np.array([0, delta]))) / (2 * delta)
        return np.array([dx, dy])

    def move(self):
        step_size = 0.1
        max_iter = self.max_iter
        threshold = self.threshold

        for _ in range(max_iter):
            gradient = self.gradient(self.start)

            # Update position based on gradient descent
            self.start -= step_size * gradient

            # Append the new position to the list of positions
            self.positions.append(self.start.copy())

            # Check if the distance to the goal is below the threshold
            if np.linalg.norm(self.start - self.goal) < threshold:
                break

    def plot_movement(self):
        positions = np.array(self.positions)
        plt.clf()  # Clear previous plot
        plt.plot(positions[:, 0], positions[:, 1], '-o')  # Plot x vs y
        plt.scatter(self.goal[0], self.goal[1], color='red', label='Goal')  # Plot goal position
        for i, obstacle in enumerate(self.obstacles):
            plt.scatter(obstacle[0], obstacle[1], color='black', label=f'Obstacle {i+1}')  # Plot obstacles
            circle = plt.Circle((obstacle[0], obstacle[1]), self.rep_radii[i], color='black', fill=False)
            plt.gca().add_artist(circle)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Dot Movement')
        plt.legend()
        plt.grid(True)
        plt.pause(0.001)  # Non-blocking pause to update the plot


if __name__ == '__main__':

    rospy.init_node("apf")

	sub_current_pose = rospy.Subscriber('/Pos', PoseStamped, set_current_pose)
	sub_goal_pose = rospy.Subscriber('/Target_Pos', PoseStamped, set_goal_pose)
    control_pub = rospy.Publisher('/Control_Pose',PoseStamped, queue_size=10)

    start_position = [0.0, 0.0]  # Initial position
    goal_x = float(input("Enter goal x coordinate: "))
    goal_y = float(input("Enter goal y coordinate: "))
  
    obstacles = np.array([[1, 2], [4, 1]])  # Convert to numpy array for easier calculations
    rep_radii = [1, 1.5]  # Radius for each obstacle

    goal = [goal_x, goal_y]
    my_dot = Dot(start_position, goal, obstacles, rep_radii)


    plt.ion()  # Turn on interactive mode

    while True:
       
        my_dot.move()

        my_dot.plot_movement()
