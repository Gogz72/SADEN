#!/usr/bin/python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy import interpolate

def successors(node):
	global Map
	List_successors=[]
	for i in -1,0,1:
		for j in -1,0,1:
			for k in -1,0,1:
				if i==0 and j==0 and k==0:
					continue
			
				if node.position[0]+i<0 or node.position[1]+j<0 or node.position[2]+k<0 or node.position[0]+i>=Map.shape[0] or node.position[1]+j>=Map.shape[1] or node.position[2]+k>=Map.shape[2]:
					#print("Out of range",node.position[0]+i,node.position[1]+j,node.position[2]+k)
					continue
				if Map[node.position[0]+i,node.position[1]+j,node.position[2]+k]>0:
					List_successors.append(node_class([node.position[0]+i,node.position[1]+j,node.position[2]+k],None,None,None,node,Map[node.position[0]+i,node.position[1]+j,node.position[2]+k]))
					x_,y_,z_=convert_map_to_copelia([node.position[0]+i,node.position[1]+j,node.position[2]+k])
					#print("Publishing",x_,y_,z_)
					#pub.publish(Quaternion(x_,y_,z_,5))
					#rate2.sleep()


	#for node in List_successors:
	#    if node.position==node.parent:
	#        List_successors.remove(node) #remove the node itself
	return List_successors
	
	
class node_class:
	def __init__(self, position,g,h,f,parent,Type):
		self.parent = parent
		self.position = position
		self.g =g 
		self.h =h
		self.f =f 
		self.type=Type
		

def distance(node,goal):
	return math.dist(node.position,goal.position)


def get_minimum(List):
	min_f_list=[]
	min_value_f=100
	min_value_h=100
	min_index=0
	for i in range(len(List)):
		if List[i].f < min_value_f:
			min_value_f=List[i].f
			min_index = i

	for i in range(len(List)):
		if List[i].f == min_value_f:
			min_f_list.append(i)

	for i in min_f_list:
		if List[i].h < min_value_h:
			min_value_h=List[i].h
			min_index = i
	return List[min_index],min_index


def A_star(start,goal):
	open_list = []
	closed_list = []
	open_list.append(start)
	rate.sleep()
	while len(open_list) > 0:
		parent,index=get_minimum(open_list)
		open_list.pop(index)
		childern=successors(parent)

		for child in childern:
			if child.position == goal.position:
				goal.parent=parent
				return goal

		for child in childern:
			g_cost=parent.g+distance(parent,child)
			h_cost=distance(child,goal)
			#print(child.position,goal.position)
			#print(abs(child.position[2]-goal.position[2]))
			z_cost=abs(child.position[2]-goal.position[2])
			#print("z_cost",z_cost)
			f_cost=g_cost+h_cost+z_cost
			child.g=g_cost
			child.h=h_cost
			child.f=f_cost
			flag1=0
			flag2=0
			for i in range(len(open_list)):
				if child.position==open_list[i].position:
					flag1=1
					index1=i
					break
			
			for i in range(len(closed_list)):
				if child.position==closed_list[i].position:
					flag2=1
					index2=i
					break

			if(flag2==1):
				if(child.f<closed_list[index2].f):
					closed_list[index2]=child
			elif(flag1==1):
				if(child.f<open_list[index1].f):
					open_list[index1]=child
			else:
				open_list.append(child) 
		
		closed_list.append(parent)


def path_generation(path):
	path_list_rev=[]
	path_list=[]
	while 1:
		path_list_rev.append(path.position)
		path=path.parent
		if path==None:
			break
	for i in range(len(path_list_rev)):
		path_list.append(path_list_rev[len(path_list_rev)-1-i])
	return path_list


def Draw_Horizontal(Array,Index1,Index2):   #Draws A horizontal Line from index1 to index2
	Local_Counter = Index1[1]
	while(Local_Counter <= Index2[1]):
		Array[Index1[0],Local_Counter,Index1[2]]= 0
		x_,y_,z_=convert_map_to_copelia([Index1[0],Local_Counter,Index1[2]])
		pub.publish(Quaternion(x_,y_,z_,0))
		rate3.sleep()
		Local_Counter+=1

def Draw_Vertical(Array,Index1,Index2):    #Draws A Vertical Line from index1 to index2
	Local_Counter = Index1[0]
	while(Local_Counter <= Index2[0]):
		Array[Local_Counter,Index1[1],Index1[2]]= 0
		x_,y_,z_=convert_map_to_copelia([Local_Counter,Index1[1],Index1[2]])
		pub.publish(Quaternion(x_,y_,z_,0))
		rate3.sleep()
		local_Counter+=1


def Draw(Map):    #Draws a Map
	draw_counter=0
	global x,y,z,nxy,nz
	while(draw_counter<x):
		#Draw_Horizontal(Map,[draw_counter,0,1],[draw_counter,y-1,1])
		#Draw_Horizontal(Map,[draw_counter,0,z-1],[draw_counter,y-1,z-1])
		#Draw_Horizontal(Map,[draw_counter,0,0],[draw_counter,y-1,0])
		draw_counter=draw_counter+1
	draw_counter=0
	while(draw_counter<z):
		#Draw_Horizontal(Map,[0,0,draw_counter],[0,y-1,draw_counter])
		#Draw_Horizontal(Map,[x-1,0,draw_counter],[x-1,y-1,draw_counter])
		#Draw_Vertical(Map,[0,0,draw_counter],[x-1,0,draw_counter])       
		#Draw_Vertical(Map,[0,y-1,draw_counter],[x-1,y-1,draw_counter])

		Draw_Horizontal(Map,[nxy*2,0,draw_counter],[nxy*2,y-4,draw_counter]) ####trial boundaries
		Draw_Horizontal(Map,[nxy*4,3,draw_counter],[nxy*4,y-1,draw_counter]) ####trial boundaries
		Draw_Horizontal(Map,[nxy*6,0,draw_counter],[nxy*6,y-4,draw_counter]) ####trial boundaries
		Draw_Horizontal(Map,[nxy*8,3,draw_counter],[nxy*8,y-1,draw_counter]) ####trial boundaries
		draw_counter=draw_counter+1

def set_current_pose(data):
	global current_point,current_posestamped,start_point2
	current_posestamped=data
	start_point2=[data.pose.position.x,data.pose.position.y,data.pose.position.z]
	current_point=convert_copelia_to_map([data.pose.position.x,data.pose.position.y,data.pose.position.z])
	#print("Current Pose Set",current_point)
	#sub_current_pose.unregister()

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

def convert_copelia_to_map(copelia_pose,round_value=True):
	global nxy,nz,copelia_map_h,copelia_map_w,copelia_map_d,map_scale
	map_pose=[]
	print("copelia_pose",copelia_pose)
	x=(copelia_pose[0]+copelia_map_h/2)*nxy/map_scale
	y=(copelia_pose[1]+copelia_map_w/2)*nxy/map_scale
	z=copelia_pose[2]*nz
	if round_value:
		x=round(x)
		y=round(y)
		z=round(z)
	map_pose.append(x)
	map_pose.append(y)
	map_pose.append(z)
	# map_pose.append(round((copelia_pose[0]+copelia_map_h/2)*nxy/map_scale))
	# map_pose.append(round((copelia_pose[1]+copelia_map_w/2)*nxy/map_scale))
	# map_pose.append(round(copelia_pose[2]*nz))
	return map_pose

def convert_map_to_copelia(map_pose,round_value=True):
	global nxy,nz,copelia_map_h,copelia_map_w,copelia_map_d,map_scale
	copelia_pose=[]
	print("map_pose",map_pose)
	x=((map_pose[0]*map_scale)/(nxy))-copelia_map_h/2
	y=((map_pose[1]*map_scale)/(nxy))-copelia_map_w/2
	z=map_pose[2]/nz
	if round_value:
		x=round(x,2)
		y=round(y,2)
		z=round(z,2)
	copelia_pose.append(x)
	copelia_pose.append(y)
	copelia_pose.append(z)
	# copelia_pose.append(round(((map_pose[0]*map_scale)/(nxy))-copelia_map_h/2,2))
	# copelia_pose.append(round(((map_pose[1]*map_scale)/(nxy))-copelia_map_w/2,2))
	# copelia_pose.append(round(map_pose[2]/nz,2))
	return copelia_pose

def path_planning(Map,current_point,goal_point):
	global x,y,z,nxy,nz,current_orientation,goal_posestamped,goal_point2,start_point2
	start=node_class(current_point,0,0,0,None,2)  #position,g,h,f,parent
	start_x,start_y,start_z=start_point2#convert_map_to_copelia(current_point)
	pub.publish(Quaternion(start_x,start_y,start_z,2))
	print(start.position)
	#goal out of bounds
	if goal_point[0]>=x or goal_point[1]>=y or goal_point[2]>=z or goal_point[0]<0 or goal_point[1]<0 or goal_point[2]<0:
		print("Goal out of bounds")
		return
		#exit()
	if Map[goal_point[0],goal_point[1],goal_point[2]]==0:
		print("Goal is in obstacle")
		return
		#exit()
	goal=node_class(goal_point,0,0,0,None,3)
	goal_x,goal_y,goal_z=goal_point2#convert_map_to_copelia(goal_point)
	pub.publish(Quaternion(goal_x,goal_y,goal_z,3))
	print(goal.position)
	Map[start.position[0],start.position[1],start.position[2]]=start.type
	Map[goal.position[0],goal.position[1],goal.position[2]]=goal.type

	path = A_star(start,goal)
	final_path=path_generation(path)
	print("len1final_path",len(final_path))
	final_path.pop(0)
	final_path.insert(0,convert_copelia_to_map(start_point2,round_value=False))
	final_path.pop(-1)
	final_path.append(convert_copelia_to_map(goal_point2,round_value=False))
	print("len2final_path",len(final_path))
	print("final_path",final_path)
	#fit spline
	if len(final_path)>3:
		path=np.array(final_path)
		print(path,path.shape)
		path=np.transpose(path)
		print("path",path,path.shape)
		tck, u = interpolate.splprep([path[0],path[1],path[2]], s=1)
		u=np.linspace(0,1,len(final_path)*5)
		final_path = interpolate.splev(u, tck)
		final_path=np.transpose(final_path).tolist()
		print("final_pathspline",final_path,tck,u)
	#final_path=interpolate.splrep(final_path[:,0],final_path[:,1],final_path[:,2],s=0)
	

	orientations=[]
	current_orientation=[current_posestamped.pose.orientation.x,current_posestamped.pose.orientation.y,current_posestamped.pose.orientation.z,current_posestamped.pose.orientation.w]
	current_orientation=euler_from_quaternion(current_orientation)[2]
	print("current",current_orientation)
	goal_orientation=[goal_posestamped.pose.orientation.x,goal_posestamped.pose.orientation.y,goal_posestamped.pose.orientation.z,goal_posestamped.pose.orientation.w]
	goal_orientation=euler_from_quaternion(goal_orientation)[2]
	print("goal",goal_orientation)
	orientation_diff=goal_orientation-current_orientation
	orientation_step=orientation_diff/len(final_path)
	print("here",orientation_diff,orientation_step,len(final_path))
	for i in range(1,len(final_path)):
		#orientations.append(math.atan2(final_path[i][1]-final_path[i-1][1],final_path[i][0]-final_path[i-1][0])) #angle between two points
		print("i",i,orientation_step*i)
		orientation=current_orientation+orientation_step*i
		print(orientation)
		orientations.append(orientation)

	final_path.pop(0)
	final_path.pop(-1)

	msg = Path()
	msg.header.frame_id = "map"
	msg.header.stamp = rospy.Time.now()
	msg.poses.append(current_posestamped)
	for i in final_path:
		pose = PoseStamped()
		x_,y_,z_=convert_map_to_copelia(i)
		pose.header.frame_id = "map"
		pose.pose.position.x = x_
		pose.pose.position.y = y_
		pose.pose.position.z = z_
		quaternion=quaternion_from_euler(0,0,orientations.pop(0))
		pose.pose.orientation.x = quaternion[0]
		pose.pose.orientation.y = quaternion[1]
		pose.pose.orientation.z = quaternion[2]
		pose.pose.orientation.w = quaternion[3]
		msg.poses.append(pose)
	msg.poses.append(goal_posestamped)
	path_pub.publish(msg)

	for i in final_path[1:-1]:
		#Map[i[0],i[1],i[2]]=4
		x_,y_,z_=convert_map_to_copelia(i)
		pub.publish(Quaternion(x_,y_,z_,4))
		rate4.sleep
	
	#goal_changed=False


if __name__ == '__main__':     # Main function that is executed 

	rospy.init_node('Path_planning', anonymous=True)
	global sub_current_pose,sub_goal_pose,goal_point2,start_point2,goal_point_prev,goal_changed,counter,copelia_map_h,copelia_map_w,copelia_map_d,map_scale
	counter=0
	goal_changed=False
	goal_point_prev=None
	sub_current_pose = rospy.Subscriber('/Pos', PoseStamped, set_current_pose)
	sub_goal_pose = rospy.Subscriber('/Target_Pos', PoseStamped, set_goal_pose)
	pub = rospy.Publisher('/sim_ros_interface/map/state',Quaternion, queue_size=27000)
	control_pub = rospy.Publisher('/Conrol_Pose',PoseStamped, queue_size=10)
	reset_pub=rospy.Publisher('/sim_ros_interface/map/reset',Bool,queue_size=10)
	path_pub=rospy.Publisher('/path',Path,queue_size=10)
	rate = rospy.Rate(0.5) 
	rate2= rospy.Rate(1000)
	rate3= rospy.Rate(300)
	rate4= rospy.Rate(5)
	map_scale=1
	copelia_map_h, copelia_map_w, copelia_map_d = 10*map_scale, 10*map_scale, 4
	nxy = 2
	nz = 4
	x = (nxy*10)+1
	y = (nxy*10)+1
	z = nz
	current_point=None
	start_point2=None
	goal_point=None
	goal_point2=None
	while (not rospy.is_shutdown()) and current_point==None or goal_point==None:
		#print("Waiting for Pose")
		rate2.sleep()
		#print(current_point,goal_point)
		pass
	pub.publish(Quaternion(0,0,0,1))
	rate.sleep()
	Map = np.ones((x,y,z))
	Draw(Map)
	while not rospy.is_shutdown():
		if goal_changed and counter >=10:
			reset_pub.publish(True)
			path_planning(Map,current_point,goal_point)
			counter=0
			goal_changed=False
			sub_goal_pose = rospy.Subscriber('/Target_Pos', PoseStamped, set_goal_pose)
		rate.sleep()
