import numpy as np
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Int32 , String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy import interpolate
import random

pose = [0,0,0,0,0,0]



def pi_2_pi(pose_angles):
    #A function to wrap the angle between -pi and pi (for numerical stability)
    pose_angles[3] = (pose_angles[3] + np.pi) % (2 * np.pi) - np.pi
    pose_angles[4] = (pose_angles[3] + np.pi) % (2 * np.pi) - np.pi
    pose_angles[5] = (pose_angles[3] + np.pi) % (2 * np.pi) - np.pi

    return pose_angles

def is_valid_move(x, y, grid):
    # Check if the move is within the boundaries of the grid
    return 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0

def generate_random_path(grid):
    # Randomly generate a path in the grid
    path = [(0, 0)]  # Starting position
    current_position = (0, 0)

    while current_position != (len(grid) - 1, len(grid[0]) - 1):
        possible_moves = [
            (current_position[0] + 1, current_position[1]),
            (current_position[0], current_position[1] + 1)
        ]

        valid_moves = [
            move for move in possible_moves if is_valid_move(move[0], move[1], grid)
        ]

        if not valid_moves:
            break

        next_position = random.choice(valid_moves)
        path.append(next_position)
        current_position = next_position

    return path


def odom_callback (msg):
    global pose

    pose = msg.data


def qr_callback (msg) :
    prompt = msg.data

def rotate_pose_90_degrees(pose_stamped):
    # Assuming the input pose_stamped has the representation (x, y, z, alpha, beta, gamma)
    current_gamma = pose_stamped.gamma

    # Rotate 90 degrees around the z-axis
    new_gamma = current_gamma + 1.5708  # 90 degrees in radians

    # Update the pose_stamped with the new gamma angle
    pose_stamped.gamma = new_gamma

    return pose_stamped



def main():
    global pose

    rospy.init_node("random_pp")

    rospy.Subscriber("/Pos",PoseStamped , odom_callback)
    control_pub = rospy.Publisher('/Conrol_Pose',PoseStamped, queue_size=10)
    rospy.Subscriber("/qr_data",String, qr_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
    
      proximity_msg = rospy.wait_for_message("/sim_ros_interface/proximity_sensor/state", Int32)

      if proximity_msg.data != 0 :
           pose = rotate_pose_90_degrees(pose)
           pose = pi_2_pi(pose)

      else:
          
          


      rate.sleep


    




if __name__ == "__main__":
    main()