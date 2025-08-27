#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf
import sys

def send_goal(x, y, yaw):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo("Sending goal: x={}, y={}, yaw={}".format(x, y, yaw))
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Result: {}".format(client.get_result()))

if __name__ == "__main__":
    rospy.init_node("send_goal")

    if len(sys.argv) == 4:
      try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
      except ValueError:
        rospy.logger("Arguments must be numbers: x y yaw")
        exit(1)
    else:
      x = rospy.get_param("~goal_x", 0.0)
      y = rospy.get_param("~goal_y", 2.0)
      yaw = rospy.get_param("~goal_yaw", 0.0)

    try:
        send_goal(x, y, yaw)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
