#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
#from simple_pid import PID


def params_callback(msg):
    global parameters
    parameters = msg.data

def setpoint_callback(msg):
   global X, Y, Z
   Array = msg.data
   X = Array[0]
   Y = Array[1]
   Z = Array[2]

def odom_callback (msg):
   global odom
   odom = msg.data

def euler_callback (msg):
   global euler
   euler = msg.data
   
   
   

def PID_Cont (kp, kd, ki, target, actual, errorsum, lasterror) :
    error = target - actual
    errorsum += error
    errorDiff = error - lasterror

    # PID control law
    Output = (kp * error) + (ki * errorsum) + (kd * errorDiff)

    # Update last error for the next iteration
    lastError = error

    return Output, errorsum, lastError



def main():
    
    rospy.init_node("Drone")

    rospy.Subscriber("/Params", Float32MultiArray, params_callback)
    
    rospy.Subscriber("/Setpoint", Float32MultiArray, setpoint_callback)

    rospy.Subscriber("/Odom", Float32MultiArray , odom_callback)

    rospy.Subscriber("/Euler", Float32MultiArray, euler_callback)

    output_pub = rospy.Publisher("/Output" , Float32MultiArray , queue_size=10)
    
    output = []

    output_msg = Float32MultiArray()
    
    errorsum_vertical = 0.0
    lastError_vertical = 0.0
    thrust = 0.0

    lastError_pitch = 0.0
    errorsum_pitch = 0.0
    pitch_stabilization = 0.0

    errorsum_roll = 0.0
    lastError_roll = 0.0
    roll_stabilization = 0.0



    rate = rospy.Rate(10)  # Adjust the rate as needed
    
    
    while not rospy.is_shutdown():
     global X, Y, Z, parameters, odom, euler

     if errorsum_vertical > 100.0:
         errorsum_vertical = 0.0

     if errorsum_roll > 100.0:
         errorsum_roll = 0.0

     if errorsum_pitch > 100.0:
         errorsum_pitch = 0.0

     if 'X' in globals() and 'Y' in globals() and 'Z' in globals() \
            and 'parameters' in globals() and 'odom' in globals() and 'euler' in globals():
         
         thrust, errorsum_vertical, lastError_vertical = PID_Cont(parameters[16], parameters[17], parameters[18], Z, odom[2], errorsum_vertical, lastError_vertical)

         # Roll stabilization
         roll_stabilization, errorsum_roll, lastError_roll = PID_Cont(parameters[5], parameters[6], parameters[7], 0, euler[0], errorsum_roll, lastError_roll)

         # Pitch stabilization
         pitch_stabilization, errorsum_pitch, lastError_pitch = PID_Cont(parameters[8], parameters[9], parameters[10], 0, euler[1], errorsum_pitch, lastError_pitch)
         
         thrust += parameters[0] * roll_stabilization + parameters[1] * pitch_stabilization

         if not output:
                output.append(thrust)

         else:
                output[0] = thrust

        
         output_msg.data = output
         output_pub.publish (output_msg)

     else:
        print("Variables not defined yet.")
     
    

     rate.sleep()



if __name__ == '__main__':
 
 main()
 