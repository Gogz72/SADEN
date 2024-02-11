#!/usr/bin/python3
import math
from simple_pid import PID
import numpy as np
import rospy
from geometry_msgs.msg import Pose,PoseStamped,Quaternion
import time

def pi_2_pi(angle):
    #A function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Point_class:
    def __init__(self,x:float,y:float,z:float):
        self.x=x
        self.y=y
        self.z=z

class Quaternion_class:
    def __init__(self,x:float,y:float,z:float,w:float=None): 
        #If the Inputs were (x,y,z), then we are using Euler Cooridnates. If (x,y,z,w) then the input is Quaternion  
        if w != None:
            self.x=x
            self.y=y
            self.z=z
            self.w=w
        else: #Euler to Quaternion
            self.x = math.sin(x/2) * math.cos(y/2) * math.cos(z/2) - math.cos(x/2) * math.sin(y/2) * math.sin(z/2)
            self.y = math.cos(x/2) * math.sin(y/2) * math.cos(z/2) + math.sin(x/2) * math.cos(y/2) * math.sin(z/2)
            self.z = math.cos(x/2) * math.cos(y/2) * math.sin(z/2) - math.sin(x/2) * math.sin(y/2) * math.cos(z/2)
            self.w = math.cos(x/2) * math.cos(y/2) * math.cos(z/2) + math.sin(x/2) * math.sin(y/2) * math.sin(z/2)
        #print(self.x, self.y, self.z, self.w)
    def quaternion(self):
        return np.array([self.x, self.y, self.z, self.w])
    def euler_zyx(self):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x=self.x
        y=self.y
        z=self.z
        w=self.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return np.array([roll_x, pitch_y, yaw_z]) #In radians

class Pose_class:
    def __init__(self,position:Point_class=None, orientation:Quaternion_class=None):
        self.position=position
        self.orientation=orientation
    def get_position(self):
        return np.array([self.position.x, self.position.y, self.position.z])

#Setting Points 
current_state = Pose_class(Point_class(0,0,0), Quaternion_class(0,0,0))
goal_point = Pose_class(Point_class(0,0,0), Quaternion_class(0,0,0))

def set_goal_pose(goal_msg): # Pose msg
    """
        This is the callback of a ros subscriber,
        It receives the Goal Pose and store it in a global variable (goal_point)
    """
    global goal_point
    #print(goal_msg)
    goal_msg=goal_msg.pose
    #print(goal_msg)
    x,y,z,w=goal_msg.orientation.x,goal_msg.orientation.y,goal_msg.orientation.z,goal_msg.orientation.w
    goal_point = Pose_class(goal_msg.position,Quaternion_class(x,y,z,w))
    #goal_point.position.z=2
    #print("position: ",goal_point.get_position())
    #print("orientation quaternion: ",goal_point.orientation.quaternion())
    #print("orientation euler_zyx: ",goal_point.orientation.euler_zyx())
    #print("goal_point set")

def set_current_pose(current_msg): # Pose msg
    """
        This is the callback of a ros subscriber,
        It receives the Current Drone Pose and store it in a global variable (current_state)
        Then do one iteration in the control loop
    """
    global current_state
    current_msg=current_msg.pose
    x,y,z,w=current_msg.orientation.x,current_msg.orientation.y,current_msg.orientation.z,current_msg.orientation.w
    #print(current_msg)
    #print(x,y,z,w)
    current_state = Pose_class(current_msg.position,Quaternion_class(x,y,z,w))
    #print("position: ",current_state.get_position())
    #print("orientation quaternion: ",current_state.orientation.quaternion())
    #print("orientation euler_zyx: ",current_state.orientation.euler_zyx())
    #print("current_state set")
    control_loop()

#PID Controllers initialization
z_dot_pid = PID(100, 20, 0.0, setpoint=0)

x_dot_pid = PID(0.4, 0.001, 0, setpoint=0)
x_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone pitch angle
y_dot_pid = PID(0.4, 0.001, 0, setpoint=0)
y_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone roll angle

yaw_dot_pid = PID(2, 0, 0, setpoint=0)
yaw_dot_pid.output_limits = (-0.5, 0.5)       #maximum drone yaw rate

roll_pid = PID(0.1, 0.001, 0.05, setpoint=0)
pitch_pid = PID(0.1, 0.001, 0.05, setpoint=0)

x_prev=0
y_prev=0
z_prev=0
yaw_prev=0
time_prev=time.time()
def control_loop():
    """
        This function do the control loop for x,y,z positions and roll,pitch,yaw angles
        using 6 pid controllers
        and publish the control commands(TYPR) to ros
        Inputs: goal point pose (x,y,z,roll,pitch,yaw)
                drone current pose (x,y,z,roll,pitch,yaw)
        Outputs: publish TYPR control commands
    """
    global current_state, goal_point, x_prev, y_prev, time_prev, yaw_prev, z_prev
    time_now = time.time()
    dt = time_now - time_prev

    theta=current_state.orientation.euler_zyx()[2]      #Drone Orientation

    dx = (current_state.position.x-x_prev)/dt
    dy = (current_state.position.y-y_prev)/dt
    dz = (current_state.position.z-z_prev)/dt
    x_prev = current_state.position.x
    y_prev = current_state.position.y
    z_prev = current_state.position.z
    yaw_dot= (theta-yaw_prev)/dt
    yaw_prev = theta
    time_prev = time_now
    #print("dt = ",dt)
    print("dx = ",dx,"dy = ",dy,"dz = ",dz,"dt = ",dt,"yaw_dot = ",yaw_dot)
    #Height Controller
 #   throttle_pid.setpoint= goal_point.position.z
  #  throttle_ = throttle_pid(current_state.position.z)

    #Yaw angle Controller
    phi= pi_2_pi(goal_point.orientation.euler_zyx()[2] - current_state.orientation.euler_zyx()[2]) #In radians

    yaw_dot_pid.setpoint = phi

    print("yaw_dot_pid.setpoint = ",yaw_dot_pid.setpoint)
    yaw_ = yaw_dot_pid(yaw_dot)
    #print("phi = ",phi)
    
    #roll angle Controller
    alpha= pi_2_pi(-current_state.orientation.euler_zyx()[0]) #In radians
    roll_ = -roll_pid(alpha)
    #print("roll = ",roll_)
    
    #pitch angle Controller
    beta= pi_2_pi(-current_state.orientation.euler_zyx()[1]) #In radians
    pitch_ = -pitch_pid(beta)
    #print("pitch = ",pitch_)

    #x,y position Controller
    delta = goal_point.get_position()-current_state.get_position()  #Difference in positions in x,y,z between goal point and current state point
                                                                    # with reference to the global frame
    x_=delta[0]*math.cos(theta)+delta[1]*math.sin(theta)    #Get the difference in x with reference to the drone frame
    y_=-delta[0]*math.sin(theta)+delta[1]*math.cos(theta)   #Get the difference in y with reference to the drone frame
    #print("delta = ",delta)

    x_dot = dx*math.cos(theta) + dy*math.sin(theta)
    y_dot = -dx*math.sin(theta) + dy*math.cos(theta)
    #x,y velocity Controller
    x_dot_pid_output = x_dot_pid(x_dot)
    y_dot_pid_output = y_dot_pid(y_dot)
    throttle_ = z_dot_pid(dz)
    
    x_dot_pid.setpoint=x_*0.3
    y_dot_pid.setpoint=y_*0.3
    z_dot_pid.setpoint=delta[2]
    if x_>0.7:
        x_dot_pid.setpoint = 0.4
    elif x_<-0.7:
        x_dot_pid.setpoint = -0.4
    if y_>0.7:
        y_dot_pid.setpoint = 0.4
    elif y_<-0.7:
        y_dot_pid.setpoint = -0.4
    if delta[2]>0.8:
        z_dot_pid.setpoint = 10
    elif delta[2]<-0.8:
        z_dot_pid.setpoint = -10
    print("x_dot_pid.setpoint = ",x_dot_pid.setpoint,"y_dot_pid.setpoint = ",y_dot_pid.setpoint,"z_dot_pid.setpoint = ",z_dot_pid.setpoint)

    roll_pid.setpoint =  y_dot_pid_output       #setpoint of the roll angle controller
    pitch_pid.setpoint = -x_dot_pid_output      #setpoint of the pitch angle controller

    #print("pitch_command = ",pitch_)
    #print("roll_command = ",roll_)
    #print("throttle_command = ",throttle_)
    #print("yaw_command = ",yaw_)
    
    print("Error:","(x,y,z):",delta,"(roll,pitch,yaw):",goal_point.orientation.euler_zyx()-current_state.orientation.euler_zyx())

    control_commands=Quaternion(pitch_,roll_,throttle_,yaw_)
    print("Control Commands: ",control_commands)
    pub.publish(control_commands)

#Initialize ros node, publisher and subscribers
rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('/control_commands', Quaternion, queue_size=10)
rate = rospy.Rate(10)
sub_current_pose = rospy.Subscriber('/Pos', PoseStamped, set_current_pose)
sub_goal_pose = rospy.Subscriber('/Conrol_Pose', PoseStamped, set_goal_pose)

while not rospy.is_shutdown():
    rate.sleep()