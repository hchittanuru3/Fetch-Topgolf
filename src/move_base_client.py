#!usr/bin/env/python

import rospy
import actionlib
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import transformations


class MoveBaseClient(object):
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        print(x, y, theta)
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        (move_goal.target_pose.pose.orientation.x,
         move_goal.target_pose.pose.orientation.y,
         move_goal.target_pose.pose.orientation.z,
         move_goal.target_pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, math.radians(theta))
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('move_base_node')
    move_base = MoveBaseClient()
    exploration_poses = [(0, 0, 45), (1, 4.5, -90), (0, 8, -135), (4, 8, 180), (8, 8, 135), (8, 4, 90), (8, 0, 45), (4, 0, 0)] #theta = -90 is facing +x, theta = 0 is facing +y
    for i in exploration_poses:
        move_base.goto(*i)
        print("Reached", i)
        quit()