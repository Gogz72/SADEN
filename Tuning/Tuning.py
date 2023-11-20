#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

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

kp_rate_roll = 10.0
kd_rate_roll = 0
ki_rate_roll = 0.6

def main():
    rospy.init_node("Tuning")
    parameter_pub = rospy.Publisher("Params", Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        parameters = [kp_stabilize_roll, kp_stabilize_pitch, kp_stabilize_yaw, kp_pos_xy, kp_rate_roll, kd_rate_roll,
                      ki_rate_roll, kp_rate_pitch, kd_rate_pitch, ki_rate_pitch, kp_rate_yaw, kd_rate_yaw,
                      ki_rate_yaw, kp_rate_roll, kd_rate_roll, ki_rate_roll]

        # Create a Float32MultiArray message
        array_msg = Float32MultiArray()
        array_msg.data = parameters

        # Publish the message
        parameter_pub.publish(array_msg)
        rate.sleep()
if __name__ == '__main__':
    main()
