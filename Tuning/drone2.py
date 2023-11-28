#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

import numpy as np

# PID parameters
p_param = 1.0
i_param = 0.1 
d_param = 0.5

# Other gains
v_param = 0.1
alpha_gain = 0.25
beta_gain = -0.25
rot_gain = 0.1

# Store previous errors
prev_e = 0
prev_alpha_e = 0
prev_beta_e = 0 
prev_euler = 0

def control_loop():

    # Get target and drone positions
    target_pos = get_target_pos() 
    drone_pos = get_drone_pos()

    # Vertical control
    e = target_pos[2] - drone_pos[2]
    cum_error = cum_error + e
    p_term = p_param * e
    i_term = i_param * cum_error 
    d_term = d_param * (e - prev_e)

    thrust = 5.45 + p_term + i_term + d_term 

    # Horizontal control
    alpha_e, beta_e = get_horizontal_errors()
    alpha_corr = alpha_gain * alpha_e + (alpha_e - prev_alpha_e)
    beta_corr = beta_gain * beta_e + (beta_e - prev_beta_e)

    # Rotational control
    euler, rot_corr = get_orientation_error()

    # Send motor commands
    motor_msg = Float32MultiArray()
    motor_msg.data = [thrust*(1-alpha_corr+beta_corr+rot_corr), ...]

    pub.publish(motor_msg)

    # Update previous errors
    prev_e = e
    prev_alpha_e = alpha_e 
    prev_beta_e = beta_e
    prev_euler = euler

def run():
    rospy.init_node('control_node')
    global pub
    pub = rospy.Publisher('/drone/motors', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(50) # 50Hz

    while not rospy.is_shutdown():
        control_loop()
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass