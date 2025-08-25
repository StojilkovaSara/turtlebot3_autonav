#!/usr/bin/env python
import rospy
import actionlib
import yaml
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def load_waypoints(file_path):
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)
    return data["waypoints"]

def move_to_goal(client, x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # yaw to quaternion
    q = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation = Quaternion(*q)

    rospy.loginfo("Sending goal: x={}, y={}, yaw={}".format(x, y, yaw))
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Goal reached!")

def main():
    rospy.init_node("multi_goal_nav")

    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun my_package multi_goal_nav.py path_file.yaml")
        sys.exit(1)

    path_file = sys.argv[1]
    waypoints = load_waypoints(path_file)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    for wp in waypoints:
        x, y, yaw = wp
        move_to_goal(client, x, y, yaw)

    rospy.loginfo("Finished all waypoints!")

if __name__ == "__main__":
    main()

