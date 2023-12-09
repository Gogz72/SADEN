import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import cv2
from cv_bridge import CvBridge
from pyzbar import pyzbar

def lane_callback(msg):
    global i
    i=msg

class CameraSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.callback)
        self.qr_data_pub = rospy.Publisher("/qr_data", String, queue_size=10)
        self.count_pub = rospy.Publisher("/count", Int32, queue_size=10)
        self.lane_sub = rospy.Subscriber("/lane", String, lane_callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except rospy.ServiceException as e:
            print(e)

        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #blur = cv2.GaussianBlur(gray,(5,5),0)

        #thresh = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,11,2)

        # Find and decode QR codes in the image
        decoded_objs = pyzbar.decode(cv_image)

        # Loop over all the decoded objects
        qr_count = 0
        for decoded_obj in decoded_objs:
            # Extract the QR code's data
            global qr_data
            qr_data = decoded_obj.data.decode('utf-8')

            # Publish the QR code's data to a ROS topic
            self.qr_data_pub.publish(qr_data)

            # Print the QR code's data
            # print("QR Code data:", qr_data)

            # Draw a rectangle around the QR code in the image
            x, y, w, h = decoded_obj.rect
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Put the QR code's data on the image
            cv2.putText(cv_image, qr_data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if qr_data == i:
              self.qr_data_pub.publish(i)
              
            qr_count += 1 
            print(qr_count)

        # Publish the QR code count to a ROS topic
        self.count_pub.publish(qr_count)

        # Display the image
        cv2.imshow("Camera Stream", cv_image)
        #cv2.imshow("with filter",blur)
        cv2.waitKey(1)


def main():
    rospy.init_node("camera_subscriber", anonymous=True)
    camera_subscriber = CameraSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    i=0
    main()