#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray , Float32

# parameters
kp_stabilize_roll = 4.5
kp_stabilize_pitch = 4.5
kp_stabilize_yaw = 6.0
kp_pos_xy = 0.1

kp_rate_roll = 0.135
kd_rate_roll = 0.135
ki_rate_roll = 0.00360

kp_rate_pitch = 0.135
kd_rate_pitch = 0.135
ki_rate_pitch = 0.00360

kp_rate_yaw = 1
kd_rate_yaw = 0.018
ki_rate_yaw = 0.0

kp_rate_throttle = 10.0
kd_rate_throttle = 0
ki_rate_throttle = 0.6

kp_throttle = 1.0
kd_throttle = 0.0
ki_throttle = 0.08

def main():
    rospy.init_node("Tuning")

    parameter_pub = rospy.Publisher('/Params', Float32MultiArray, queue_size=10)
    setpoint_pub = rospy.Publisher('/Setpoint', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(10)

    parameters_msg = Float32MultiArray()
    parameters_msg.data = [
            kp_stabilize_roll, kp_stabilize_pitch, kp_stabilize_yaw, kp_pos_xy,
            kp_rate_roll, kd_rate_roll, ki_rate_roll,
            kp_rate_pitch, kd_rate_pitch, ki_rate_pitch,
            kp_rate_yaw, kd_rate_yaw, ki_rate_yaw,
            kp_rate_throttle, kd_rate_throttle, ki_rate_throttle,
            kp_throttle, kd_throttle, ki_throttle
        ]

    setpoint = [0.0, 0.0, 2.0]

    setpoint_msg = Float32MultiArray()
    setpoint_msg.data = setpoint

    while not rospy.is_shutdown():


        # Publish the message
        parameter_pub.publish(parameters_msg)
        setpoint_pub.publish(setpoint_msg)

        rate.sleep()

if __name__ == '__main__':
    main()
