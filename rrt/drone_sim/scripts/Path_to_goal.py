#!/usr/bin/python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion
import rospy
#from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

def get_path(data):
	global path, current_sub, i
	path = data.poses
	i=0
	current_sub=rospy.Subscriber('/Pos', PoseStamped, get_current_pose)
	print("path: ", path)

def get_current_pose(data):
	global current_pose
	global path
	global i
	global current_sub
	current_pose = data
	#print("current_pose: ", current_pose)
	current_point = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z]
	print("i: ", i)
	try:
		next_point = [path[i].pose.position.x, path[i].pose.position.y, path[i].pose.position.z]
		print("current_point: ", current_point)
		print("next_point: ", next_point)
		print("i: ", i)
		dist=math.dist(next_point,current_point)
		if dist < 1:
			print("distance: ", dist, " i: ", i)
			i+=1
		print("path",len(path))
		pub.publish(path[i])
		x_,y_,z_ = path[i].pose.position.x, path[i].pose.position.y, path[i].pose.position.z
		point_pub.publish(Quaternion(x_,y_,z_,6))
	#if i == len(path):
	except:
		print("Last point reached")
		pub.publish(path[-1])
		x_,y_,z_ = path[-1].pose.position.x, path[-1].pose.position.y, path[-1].pose.position.z
		point_pub.publish(Quaternion(x_,y_,z_,6))
		#i=0
		#current_sub.unregister()
	#print("current_pose: ", current_pose)


if __name__ == '__main__':  
	global path
	rospy.init_node('Path_to_Goal', anonymous=True)
	rospy.Subscriber('/path', Path, get_path)
	point_pub = rospy.Publisher('/sim_ros_interface/map/state',Quaternion, queue_size=27000)
	pub=rospy.Publisher('/Conrol_Pose', PoseStamped, queue_size=10)
	path = []  
	current_pose = []
	i=0
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#we have a list of nodes we want to publish
		rate.sleep()

