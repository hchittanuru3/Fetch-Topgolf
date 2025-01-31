#!usr/bin/env/python

import rospy
import actionlib

from control_msgs.msg import PointHeadAction, PointHeadGoal

class PointHeadClient(object):
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('point_head_node')
    head_action = PointHeadClient()
    head_action.look_at(1.7, 0.0, 0.0, "base_link")

