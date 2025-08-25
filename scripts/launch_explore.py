#!/usr/bin/env python
import rospy
import roslaunch
import time

def main():
    rospy.init_node("delayed_explore_launcher")

    delay = rospy.get_param("~delay", 10)  # seconds
    launch_file = rospy.get_param("~launch_file", 
                                  "$(find explore_lite)/launch/explore.launch")

    rospy.loginfo("Waiting {} seconds before launching explore_lite...".format(delay))
    time.sleep(delay)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

    rospy.loginfo("explore_lite launched.")
    rospy.spin()

if __name__ == "__main__":
    main()

