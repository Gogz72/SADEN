#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert the image to grayscale for edge detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Canny edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Calculate distance between contours
        if len(contours) > 1:
            distances = [cv2.arcLength(contour, True) for contour in contours]
            avg_distance = sum(distances) / len(distances)
            
            # Display information about the distance
            print(f"Average distance between contours: {avg_distance}")
            
            # Draw little squares on the image
            for contour in contours:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                cv2.drawContours(cv_image, [approx], 0, (0, 255, 0), 2)
                
        # Display the resulting image
        cv2.imshow("Object Detection", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print(f"Error processing image: {e}")

def main():
    rospy.init_node('object_detection_node', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
