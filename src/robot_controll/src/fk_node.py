#!/usr/bin/env python3
import rospy
import numpy as np
import pinocchio as pin
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from tf.transformations import quaternion_from_matrix

# -----------------------------
# 加载机器人模型
# -----------------------------
URDF_PATH = "/home/leon-xu/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

# 获取末端执行器
ee_joint_name = rospy.get_param("~ee_joint_name", "Joint6")
ee_joint_id = model.getJointId(ee_joint_name)
base_frame = rospy.get_param("~ee_frame_id", "base_link")

# 模型关节
model_joint_names = model.names[1:]  # 跳过 universe
link_names = model.names[1:]

def js_cb(js: JointState):
    """接收 /joint_states → 计算 FK 并发布 link 位姿和末端位姿"""
    try:
        # --- JointState → Pinocchio q 向量
        name2pos = dict(zip(js.name, js.position))
        q = np.array([float(name2pos.get(n, 0.0)) for n in model_joint_names])

        # --- 正运动学
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        # # --- 构建 PoseArray
        # pose_array = PoseArray()
        # pose_array.header.stamp = rospy.Time.now()
        # pose_array.header.frame_id = base_frame

        # for i, name in enumerate(link_names):
        #     if i == 0:
        #         continue  # 跳过 universe joint
        #     oMi = data.oMi[int(i)]
        #     T = np.eye(4)
        #     T[:3, :3] = oMi.rotation
        #     T[:3, 3] = oMi.translation
        #     qx, qy, qz, qw = quaternion_from_matrix(T)

        #     pose = Pose()
        #     pose.position.x, pose.position.y, pose.position.z = oMi.translation
        #     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        #     pose_array.poses.append(pose)

        # pub_links.publish(pose_array)

        # --- 末端执行器位姿
        oMi = data.oMi[ee_joint_id]
        T = np.eye(4)
        T[:3, :3] = oMi.rotation
        T[:3, 3] = oMi.translation
        qx, qy, qz, qw = quaternion_from_matrix(T)

        ee_msg = PoseStamped()
        ee_msg.header.stamp = rospy.Time.now()
        ee_msg.header.frame_id = base_frame
        ee_msg.pose.position.x, ee_msg.pose.position.y, ee_msg.pose.position.z = oMi.translation
        ee_msg.pose.orientation.x, ee_msg.pose.orientation.y, ee_msg.pose.orientation.z, ee_msg.pose.orientation.w = qx, qy, qz, qw
        pub_ee.publish(ee_msg)

    except Exception as e:
        rospy.logerr_throttle(1.0, f"[FK Node] Error: {e}")

def main():
    global pub_ee, pub_links
    rospy.init_node("fk_node")
    pub_ee = rospy.Publisher("/motion_kinematics/ee_pose", PoseStamped, queue_size=10)
    # pub_links = rospy.Publisher("/motion_kinematics/link_poses", PoseArray, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, js_cb, queue_size=10)
    rospy.loginfo("✅ fk_node started: publishing /motion_kinematics/ee_pose and /motion_kinematics/link_poses")
    rospy.spin()

if __name__ == "__main__":
    main()
