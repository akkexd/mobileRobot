#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations

def send_goal(x, y, yaw):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('send_goal', anonymous=True)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0

    # Quaternion for orientation
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]

    rospy.sleep(1)
    pub.publish(goal)
    rospy.loginfo("Goal sent to robot: x={}, y={}, yaw={}".format(x, y, yaw))

if __name__ == "__main__":
    initial_x = 0.138
    initial_y = -0.016
    initial_yaw = -0.061
    send_goal(initial_x, initial_y, initial_yaw)

