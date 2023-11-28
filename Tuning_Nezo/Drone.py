#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import AttitudeTarget
#from geometry_msgs.msg import PoseStamped
# Import necessary ROS messages for attitude control
#from simple_pid import PID


beta_target_pitch = 0.0
gamma_target_yaw = 0.0
alpha_target_roll = 0.0
x = 0.0
y = 0.0
z = 0.0

def params_callback(msg):
    global parameters
    parameters = msg.data

def setpoint_callback(msg):
    global beta_target_pitch 
    global gamma_target_yaw 
    global alpha_target_roll 
    global x, y, z
    Array = msg.data
    x = Array[0]
    y = Array[1]
    z = Array[2]
    alpha_target_roll = Array[3]
    beta_target_pitch = Array[4]
    gamma_target_yaw = Array[5]


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
    
    output = [0.0,0.0,0.0,0.0]

    output_msg = Float32MultiArray()
    
    errorsum_thrust = 0.0
    lasterror_thrust = 0.0

    errorsum_roll = 0.0
    lasterror_roll = 0.0

    errorsum_pitch = 0.0
    lasterror_pitch = 0.0

    errorsum_yaw = 0.0
    lasterror_yaw = 0.0

    errorsum_x = 0.0
    lasterror_x = 0.0

    errorsum_y = 0.0
    lasterror_y = 0.0



    rate = rospy.Rate(10)  # Adjust the rate as needed
    
    
    while not rospy.is_shutdown():
     global x, y, z, parameters, odom, euler

     if abs(errorsum_thrust) > 80.0:
        errorsum_thrust = 0.0
        
     if abs(errorsum_roll) > 0.5:
        errorsum_roll = 0.0


     if abs(errorsum_pitch) > 0.5:
        errorsum_pitch = 0.0
        

     if abs(errorsum_yaw) > 0.5:
        errorsum_yaw = 0.0
        

     if 'x' in globals() and 'y' in globals() and 'z' in globals() \
            and 'parameters' in globals() and 'odom' in globals() and 'euler' in globals():
         
            # Update thrust and retain errorsum and lasterror
            thrust, errorsum_thrust, lasterror_thrust = PID_Cont(parameters[13], parameters[14], parameters[15], z, odom[2], errorsum_thrust, lasterror_thrust)

            # Update roll and retain errorsum and lasterror
            roll_output, errorsum_roll, lasterror_roll = PID_Cont(parameters[5], parameters[6], parameters[7], alpha_target_roll, euler[0], errorsum_roll, lasterror_roll)

            # Update pitch and retain errorsum and lasterror
            pitch_output, errorsum_pitch, lasterror_pitch = PID_Cont(parameters[8], parameters[9], parameters[10], beta_target_pitch, euler[1], errorsum_pitch, lasterror_pitch)

            # Update yaw and retain errorsum and lasterror
            yaw_output, errorsum_yaw, lasterror_yaw = PID_Cont(parameters[11], parameters[12], parameters[13], gamma_target_yaw, euler[2], errorsum_yaw, lasterror_yaw)

                        # Additional Horizontal Control Logic
            horizontal_roll_output, errorsum_x, lasterror_x = PID_Cont(parameters[3], 0.0, 0.0, x, odom[0], errorsum_x, lasterror_x)
            horizontal_pitch_output, errorsum_y, lasterror_y = PID_Cont(parameters[3], 0.0, 0.0, y, odom[1], errorsum_y, lasterror_y)

            #thrust += parameters[0] * roll_stabilization + parameters[1] * pitch_stabilization



            # Create an AttitudeTarget message and set the required fields
           # attitude_msg = AttitudeTarget()
            #attitude_msg.header.stamp = rospy.Time.now()
            #attitude_msg.type_mask = 7  # Set the mask for roll, pitch, and yaw
            output[0] = roll_output + horizontal_roll_output
            output[1] = pitch_output + horizontal_pitch_output
            output[2] = yaw_output
            output[3] = thrust

            output_msg.data = output



            output_pub.publish (output_msg)

     else:
        print("Variables not defined yet.")
     
    

     rate.sleep()



if __name__ == '__main__':
 
 main()
 