import rospy
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from opencv_apps.msg import CircleArrayStamped
#roslaunch opencv_apps hough_circles.launch image:=/head_camera/rgb/image_raw debug_view:=false
GOLF_BALL_DIAMETER = 2 #idk what this is for now

def image_callback(msg):
  try:
    '''
      bridge = CvBridge()
      opencv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
      resized_img = cv2.resize(orig, None, fx=0.5, fy=0.5)
      gray_img = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
      ret, thresh = cv2.threshold(gray, 75, 255, cv2.THRESH_BINARY)
      circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, GOLF_BALL_DIAMETER)
    '''
    circles = msg.circles
    print([(circle.center, circle.radius) for circle in circles])
  except:
    rospy.loginfo('No image message')

def start_detection_node():
  rospy.init_node('detect_circles')
  rospy.loginfo('Detect Circles Node Started')
  #rospy.Subscriber('image', Image, image_callback)
  rospy.Subscriber('/hough_circles/circles', CircleArrayStamped, image_callback)
  rospy.spin()

if __name__ == '__main__':
  start_detection_node()
