#!/usr/bin/env python3
import rospy
import numpy as np
import pinocchio as pin
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped

# 加载机器人模型
model = pin.buildModelFromUrdf(
    "/home/leon-xu/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
)
data  = model.createData()

# 末端执行器对应的 joint ID（默认 6）
ee_joint_id = int(rospy.get_param("~ee_joint_id", 6))

# 模型的关节名顺序（Pinocchio 默认从第 1 个 link 开始）
model_joint_names = model.names[1:]
frame_id = rospy.get_param("~ee_frame_id", "base_link")


def js_cb(js: JointState):
    """
    功能:
        JointState 回调函数，将 ROS 中的关节角度映射到 Pinocchio 模型顺序，
        做一次正运动学 (FK)，输出末端位姿 PoseStamped。
    输入:
        /joint_states 消息
    输出:
        /motion_kinematics/ee_pose (PoseStamped)
    """
    # 将 JointState 中的 name 和 position 对齐到模型顺序
    name2pos = dict(zip(js.name, js.position))
    q = np.array([float(name2pos.get(n, 0.0)) for n in model_joint_names])

    # 1️⃣ 正运动学
    pin.forwardKinematics(model, data, q)
    oMi = data.oMi[ee_joint_id]  # 末端相对 base_link 的位姿

    # 2️⃣ 转为 PoseStamped 格式
    T = np.eye(4)
    T[:3, :3] = oMi.rotation
    T[:3, 3] = oMi.translation
    qx, qy, qz, qw = quaternion_from_matrix(T)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(oMi.translation[0])
    msg.pose.position.y = float(oMi.translation[1])
    msg.pose.position.z = float(oMi.translation[2])
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)

    pub_ee.publish(msg)


def main():
    global pub_ee
    rospy.init_node("fk_node")


    # 发布器和订阅器
    pub_ee = rospy.Publisher("/motion_kinematics/ee_pose", PoseStamped, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, js_cb, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    main()