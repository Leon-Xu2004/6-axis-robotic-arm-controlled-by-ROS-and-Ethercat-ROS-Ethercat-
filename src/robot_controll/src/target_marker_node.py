#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker

# 全局变量
ee_pose = None

def ee_pose_callback(msg):
    """订阅机械臂末端执行器位姿"""
    global ee_pose
    ee_pose = msg

def make_6dof_marker(server, init_pose):
    """创建 6 自由度交互 Marker"""
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = init_pose.header.frame_id
    int_marker.name = "target_pose_marker"
    int_marker.description = "eRob3 Target Pose"
    int_marker.scale = 0.2
    int_marker.pose = init_pose.pose

    # 添加 6 自由度控制（旋转 + 平移）
    controls = []
    for axis, ori in [("x", (1, 0, 0)), ("y", (0, 1, 0)), ("z", (0, 0, 1))]:
        # 旋转
        rot = InteractiveMarkerControl()
        rot.name = f"rotate_{axis}"
        rot.orientation.x, rot.orientation.y, rot.orientation.z, rot.orientation.w = ori + (1.0,)
        rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(rot)

        # 平移
        move = InteractiveMarkerControl()
        move.name = f"move_{axis}"
        move.orientation.x, move.orientation.y, move.orientation.z, move.orientation.w = ori + (1.0,)
        move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(move)

    for c in controls:
        int_marker.controls.append(c)

    # 回调函数：发布 marker 的位姿
    def feedback_cb(feedback):
        pose = PoseStamped()
        pose.header = feedback.header
        pose.pose = feedback.pose
        pub.publish(pose)
        rospy.loginfo_throttle(1.0, f"[Marker] Updated pose: {pose.pose.position}")

    # 插入 marker 并应用
    server.insert(int_marker, feedback_cb)
    server.applyChanges()

if __name__ == "__main__":
    rospy.init_node("target_marker_node")

    # 订阅末端执行器位姿
    rospy.Subscriber("/motion_kinematics/ee_pose", PoseStamped, ee_pose_callback)

    # 等待一次末端姿态
    rospy.loginfo("⏳ 等待末端执行器位姿发布 (/motion_kinematics/ee_pose)...")
    try:
        ee_pose_msg = rospy.wait_for_message("/motion_kinematics/ee_pose", PoseStamped, timeout=5.0)
        rospy.loginfo("✅ 已获取末端执行器初始姿态。")
    except rospy.ROSException:
        rospy.logwarn("⚠️ 超时未收到 ee_pose，使用默认初始姿态。")
        ee_pose_msg = PoseStamped()
        ee_pose_msg.header.frame_id = "base_link"
        ee_pose_msg.pose.position.x = 0.3
        ee_pose_msg.pose.position.y = 0.0
        ee_pose_msg.pose.position.z = 0.3
        ee_pose_msg.pose.orientation.w = 1.0

    # 创建发布器与服务器
    pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)
    server = InteractiveMarkerServer("target_marker_server")

    # 创建 marker
    make_6dof_marker(server, ee_pose_msg)

    rospy.loginfo("✅ 6D Target Marker started. 拖动 marker 以控制目标位姿。")
    rospy.spin()
