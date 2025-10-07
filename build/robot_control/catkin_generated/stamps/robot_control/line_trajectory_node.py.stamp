#!/usr/bin/env python3
import rospy
import numpy as np  # type: ignore 
import pinocchio as pin
from pinocchio import SE3, Quaternion
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R, Slerp  # type: ignore

'''
功能: 在 SE3 空间内进行插值，生成从 start 到 goal 的轨迹点序列
输入:
   start (SE3) - 起始位姿
   goal  (SE3) - 目标位姿
   steps (int) - 插值点数量

输出:
   list[SE3] - 插值得到的位姿序列
'''
def interpolate_se3(start, goal, steps):

    # 生成从 start 到 goal 的等间隔插值数组 steps 个点
    pos_list = np.linspace(start.translation, goal.translation, steps)

    # 将起点/终点的旋转矩阵转化为四元数形式
    quat_start = R.from_matrix(start.rotation).as_quat()
    quat_goal = R.from_matrix(goal.rotation).as_quat()

    # 构造球面插值 (Slerp) 对象，定义在时间区间 [0, 1]
    # 输入起点四元数和终点四元数，作为插值的边界姿态
    slerp = Slerp([0, 1], R.from_quat([quat_start, quat_goal]))

    # 在 [0, 1] 区间生成 steps 个采样点，对每个采样点执行球面插值
    # rot_list 得到的是一个 Rotation 序列，每个元素是中间姿态
    rot_list = slerp(np.linspace(0, 1, steps))

    # 将位置 pos 和旋转 rot 拼接为 Pinocchio 的 SE3 对象
    return [pin.SE3(rot.as_matrix(), pos) for rot, pos in zip(rot_list, pos_list)]

'''
功能: 将轨迹序列逐点发布到 ROS 话题 /target_pose (PoseStamped)
输入:
   trajectory (list[SE3]) - 插值后的位姿序列
输出:
   无 (函数内部通过 ROS Publisher 发布消息)
'''
def publish_trajectory(trajectory):

    # 创建 /target_pose 的 Topic
    pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)

    # 等待 Publisher 与 Subscriber 建立连接
    rospy.sleep(1.0)    

    # 发布频率 (Hz)    
    rate = rospy.Rate(1)    

    # 遍历 trajectory 列表
    for i, T in enumerate(trajectory):

        # 新建 PoseStamped 消息
        msg = PoseStamped()
        # 设置时间戳
        msg.header.stamp = rospy.Time.now()
        # 参考坐标系
        msg.header.frame_id = "base_link"

        # 设置位置
        x, y, z = T.translation
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        # 设置姿态 (四元数)
        q = R.from_matrix(T.rotation).as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        # 调试信息 (位置 + 姿态四元数 + RPY 欧拉角)
        rpy = R.from_quat(q).as_euler('xyz', degrees=True)
        rospy.loginfo(f"Traj[{i}] Pos: ({x:.3f}, {y:.3f}, {z:.3f}), "
                      f"Quat: ({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}), "
                      f"RPY(deg): ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})")

        # 发布消息
        pub.publish(msg)
        rate.sleep()



# 主程序入口
if __name__ == "__main__":
    rospy.init_node("line_trajectory_node")

    # 起点位置与姿态
    p1 = np.array([0.3, 0.1, 0.4])
    q1 = Quaternion(0, 0, 0, 1)

    # 目标位置与姿态
    p2 = np.array([0.3, 0.3, 0.4])
    q2 = Quaternion(0, 1, 0, 1)

    # 构造 SE3 位姿
    T1 = SE3(q1.matrix(), p1)
    T2 = SE3(q2.matrix(), p2)

    # 生成插值轨迹 (20 个点)
    traj = interpolate_se3(T1, T2, steps=20)

    # 发布轨迹
    publish_trajectory(traj)
