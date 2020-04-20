#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from opencv_apps.msg import CircleArrayStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf import transformations
import math

GOLF_BALL_DIAMETER = 2 #idk what this is for now
GOAL_POSE = None
is_at_initial_pose = False
initial_pose = [3, 11]

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
        goal_circle_center = circles[0].center
        print(goal_circle_center)
        # convert Point2d to PoseStamped
        #publisher_node()
    except:
        rospy.loginfo('No image message')

def start_detection_node():
    rospy.init_node('detect_circles', disable_signals=True)
    rospy.loginfo('Detect Circles Node Started')
    #rospy.Subscriber('image', Image, image_callback)
    rospy.Subscriber('/hough_circles/circles', CircleArrayStamped, image_callback)
    rospy.spin()
    
def publisher_node():
    rospy.init_node('goal_publisher')
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped)

def pose_callback(pose):
    global is_at_initial_pose
    if round(pose.pose.pose.position.x - initial_pose[0], 1) == 0.0 and round(pose.pose.pose.position.y - initial_pose[1], 1) == 0.0:
        is_at_initial_pose = True

def set_initial_pose():
    init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    
    r = rospy.Rate(100) # 10hz
    while not is_at_initial_pose:
        p = PoseWithCovarianceStamped()
        while p.header.stamp == rospy.Time(0):
            p.header.stamp = rospy.Time.now()

        p.header.frame_id = 'map'
        p.pose.pose.position.x = initial_pose[0]
        p.pose.pose.position.y = initial_pose[1]
        p.pose.pose.position.z = 0

        p.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, math.radians(-90))
        
        init_pub.publish(p)
        r.sleep()
    print("done")

if __name__ == '__main__':
    #start_detection_node()
    rospy.init_node('init_pose')
    set_initial_pose()