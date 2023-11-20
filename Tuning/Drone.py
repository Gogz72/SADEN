import rospy
from std_msgs.msg import Float64MultiArray , Float64


def main():
 
 Vertical_set_pub = rospy.Publisher("Vertical_setpoint" , Float64 , queue_size=10)

 V_sp = 5

 Vertical_set_pub.publish(V_sp)