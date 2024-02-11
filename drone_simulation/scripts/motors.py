#!/usr/bin/env python3


import numpy as np
from geometry_msgs.msg import Quaternion
import rospy

gains=np.array([1,1,1,1])
biases=np.array([0,0,23.0915,0])

def control_commands_to_motor_speeds(commands):
    #print(commands)
    commands = np.array([commands.x,commands.y,commands.z,commands.w])
    #print(commands)
    pitch,roll,thrust,yaw = (commands+biases)*gains
    print('commands',pitch,roll,thrust,yaw)

    FR= thrust*(1 - pitch - roll + yaw)
    FL= thrust*(1 - pitch + roll - yaw)
    BR= thrust*(1 + pitch - roll - yaw)
    BL= thrust*(1 + pitch + roll + yaw)
    print('speeds',FR,FL,BR,BL)
    pub.publish(Quaternion(FR,FL,BR,BL))


rospy.init_node('typr_to_motors', anonymous=True) 					#Initialize ROS node
pub = rospy.Publisher('/Motor_Speeds', Quaternion, queue_size=10)   # it could be sent by any multiarray not quternion deeh 3afana 3ady
rate = rospy.Rate(10) 												# rate of publishing msg 10hz
sub = rospy.Subscriber('/control_commands', Quaternion, control_commands_to_motor_speeds)

while not rospy.is_shutdown():
    rate.sleep()
