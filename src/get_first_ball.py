import rospy
import math
from move_base_client import MoveBaseClient
from driver import set_initial_pose
from cv_bridge import CvBridge
import cv2

import tf
from sensor_msgs.msg import Image
from opencv_apps.msg import CircleArrayStamped
from geometry_msgs.msg import PoseStamped
from point_head_client import PointHeadClient

GOLF_BALL_OFFSET = .4
GOAL_CIRCLE_CENTER = None
GOAL_POSE = None
is_at_initial_pose = False
initial_pose = (2, 10)
ball_counter = 5
ball_poses = []

# Camera parameters
focal_length = 554.255
c_x = 320.5
c_y = 240.5
DEPTH = None
TF_LISTENER = None


def image_callback(msg):
    global GOAL_CIRCLE_CENTER, GOAL_POSE
    circles = msg.circles
    if len(circles) == 0 or GOAL_POSE != None:
        #print("Skipping perception")    
        return
    GOAL_CIRCLE_CENTER = circles[0].center
    depth_sub = rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, depth_callback)
    convert_image_to_global_coordinates(GOAL_CIRCLE_CENTER)

def depth_callback(image_msg):
    global GOAL_CIRCLE_CENTER, DEPTH
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg)
    #print(cv_image)
    #print(cv_image.shape)
    x, y = int(round(GOAL_CIRCLE_CENTER.x)), int(round(GOAL_CIRCLE_CENTER.y))
    DEPTH = cv_image[y, x]

def convert_image_to_global_coordinates(point):
    global DEPTH, GOAL_POSE
    if DEPTH == None:
        return
    u = point.x
    v = point.y
    x = (u - c_x) * (DEPTH/focal_length)
    y = (v - c_y) * (DEPTH/focal_length)
    GOAL_POSE = (x, y, DEPTH)

def found_goal():
    global GOAL_POSE
    point = get_transformed_point(GOAL_POSE)
    print(point)
    move_x = point.pose.position.x
    move_y = point.pose.position.y
    x_direction = math.sqrt(move_x**2 + move_y**2) - GOLF_BALL_OFFSET
    print("moving", x_direction)
    heading = math.atan2(move_y, move_x)
    move_base.goto(0, 0, math.degrees(heading), frame='base_link')
    print("Facing correct direction")
    rospy.sleep(5)
    move_base.goto(x_direction, 0, 0, frame='base_link')
    print("Done travelling")
    rospy.sleep(10)
    GOAL_POSE = None

def sweep():
    global GOAL_POSE
    head_action = PointHeadClient()
    sweep_poses = [(1, .5, 0), (1.5, .5, 0), (2, .5, 0), (2, 0, 0), (1.5, 0, 0), (1, 0, 0), (1, -.5, 0), (1.5, -.5, 0), (2, -.5, 0), (2, -1, 0), (1.5, -1, 0), (1, -1, 0)]
    count = 0
    while GOAL_POSE == None and count < len(sweep_poses):
        x, y, z = sweep_poses[count]
        head_action.look_at(x, y, z, "base_link")
        count += 1
        rospy.sleep(2)
    if GOAL_POSE == None:
        print("No ball found")
    point = get_transformed_point(GOAL_POSE)
    print(point)
    move_x = point.pose.position.x
    move_y = point.pose.position.y
    x_direction = math.sqrt(move_x**2 + move_y**2) - GOLF_BALL_OFFSET
    print("moving", x_direction)
    heading = math.atan2(move_y, move_x)
    move_base.goto(0, 0, math.degrees(heading), frame='base_link')
    print("Facing correct direction")

    quit()


def get_transformed_point(point):
    while True and TF_LISTENER != None:
        if TF_LISTENER.frameExists("base_link") and TF_LISTENER.frameExists("head_camera_rgb_optical_frame"):
            t = TF_LISTENER.getLatestCommonTime("base_link", "head_camera_rgb_optical_frame")
            p1 = PoseStamped()
            p1.header.frame_id = "head_camera_rgb_optical_frame"
            p1.pose.position.x = point[0]
            p1.pose.position.y = point[1]
            p1.pose.position.z = point[2]
            (p1.pose.orientation.x,
            p1.pose.orientation.y,
            p1.pose.orientation.z,
            p1.pose.orientation.w) = tf.transformations.quaternion_from_euler(0, 0, 0)
            p_in_base = TF_LISTENER.transformPose("/base_link", p1)
            #print "Position of the camera in the robot base:"
            #print p_in_base
            return p_in_base

if __name__ == '__main__':
    global TF_LISTENER, GOAL_POSE
    rospy.init_node('get_first_ball_node')
    TF_LISTENER = tf.TransformListener()
    #set_initial_pose()
    move_base = MoveBaseClient()
    # head_action = PointHeadClient()
    # head_action.look_at(1, 0, 0, "base_link")
    #RELATIVE POSITION
    init_pose = (0, 0, 45) #45
    move_base.goto(*init_pose, frame='base_link')
    print("Reached", init_pose)

    circle_sub = rospy.Subscriber('/hough_circles/circles', CircleArrayStamped, image_callback)
    while GOAL_POSE == None:
        continue
    found_goal()
    sweep()
    rospy.spin()