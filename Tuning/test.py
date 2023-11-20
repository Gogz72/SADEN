#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

def params_callback(msg):
    Array = msg.data
    kp_vertical = Array[2]
    print(kp_vertical)

def main():
    rospy.init_node("test")
    rospy.Subscriber("Params", Float32MultiArray, params_callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
