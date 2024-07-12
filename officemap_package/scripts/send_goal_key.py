#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations
import sys, select, termios, tty

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send_goal(x, y, yaw):
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
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

def send_initial_pose(x, y, yaw):
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = 0.0

    # Quaternion for orientation
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    initial_pose.pose.pose.orientation.x = quaternion[0]
    initial_pose.pose.pose.orientation.y = quaternion[1]
    initial_pose.pose.pose.orientation.z = quaternion[2]
    initial_pose.pose.pose.orientation.w = quaternion[3]

    rospy.sleep(1)
    pub.publish(initial_pose)
    rospy.loginfo("Initial pose set: x={}, y={}, yaw={}".format(x, y, yaw))

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('send_goal', anonymous=True)

    # Coordinates for different paths
    home_x, home_y, home_yaw = -0.003, -0.002, 0.035
    path_a_x, path_a_y, path_a_yaw = 0.915, -1.522, 3.139
    path_b_x, path_b_y, path_b_yaw = 2.242, -3.573, -1.651
    path_c_x, path_c_y, path_c_yaw = 0.404, -5.349, -3.054

    print("Press 'i' to send the robot to the initial position (Only at the start).")
    print("Press 'h' to send the robot to the home position.")
    print("Press 'a' to send the robot to Path A.")
    print("Press 'b' to send the robot to Path B.")
    print("Press 'c' to send the robot to Path C.")
    print("Press 'q' to quit.")

    while not rospy.is_shutdown():
        key = get_key()
        if key == 'i':
            send_initial_pose(home_x, home_y, home_yaw)  # Set the initial pose
        elif key == 'h':
            send_goal(home_x, home_y, home_yaw)  # Send the goal
        elif key == 'a':
            send_goal(path_a_x, path_a_y, path_a_yaw)
        elif key == 'b':
            send_goal(path_b_x, path_b_y, path_b_yaw)
        elif key == 'c':
            send_goal(path_c_x, path_c_y, path_c_yaw)
        elif key == 'q':
            break

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

