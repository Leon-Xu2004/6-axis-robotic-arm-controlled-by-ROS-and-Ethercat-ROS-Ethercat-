#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("eepath_node")
    pub = rospy.Publisher("/ee_path", Path, queue_size=10)
    path_msg = Path()
    path_msg.header.frame_id = "base_link"

    def ee_pose_cb(msg: PoseStamped):
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses.append(msg)
        pub.publish(path_msg)

    rospy.Subscriber("/ee_pose", PoseStamped, ee_pose_cb)
    rospy.spin()

if __name__ == "__main__":
    main()
