import rospy
import tf
import geometry_msgs

if __name__ == '__main__':
    rospy.init_node('tf_base_cam')
    tf_listener_ = tf.TransformListener()


    while not rospy.is_shutdown():
        if tf_listener_.frameExists("base_link") and tf_listener_.frameExists("head_camera_rgb_optical_frame"):
            t = tf_listener_.getLatestCommonTime("base_link", "head_camera_rgb_optical_frame")
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "head_camera_rgb_optical_frame"
            p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = tf_listener_.transformPose("/base_link", p1)
            print "Position of the camera in the robot base:"
            print p_in_base