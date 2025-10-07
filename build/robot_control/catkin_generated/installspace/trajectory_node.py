#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import numpy as np # type: ignore
import rospy
import pinocchio as pin
from pinocchio import SE3
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R, Slerp # type: ignore

class LineTrajectoryNode:
    

    def __init__(self):
        '''
        功能: 通过 Subscriber 运行回调函数
        start_pose_topic: 初始位置话题
        target_pose_topic: 目标位置话题
        Interpolation_Point_topic: 轨迹插值点话题
        '''
        # --- 可配置话题名与参数 ---
        self.start_pose_topic             = rospy.get_param("~start_pose_topic",             "/start_pose")
        self.target_pose_topic            = rospy.get_param("~target_pose_topic",            "/target_pose")
        self.Interpolation_Point_topic    = rospy.get_param("~Interpolation_Point_topic",    "/Interpolation_Point")  # 避免自触发回环
        self.frame_id                     = rospy.get_param("~frame_id",                     "base_link")

        # 规划参数：两种方式 1) 固定步数 2) 距离/分辨率自动算步数
        self.fixed_steps        = rospy.get_param("~fixed_steps",           0)          # >0 则优先用固定步数
        self.cart_res           = rospy.get_param("~cartesian_resolution",  0.001)       # 米/步，fixed_steps==0 时生效
        self.min_steps          = rospy.get_param("~min_steps",             2)
        self.publish_rate_hz    = rospy.get_param("~publish_rate",          50)         # 发布频率 Hz

        # —— 订阅器 —— #
        rospy.Subscriber            (self.start_pose_topic,             PoseStamped,    self.ee_cb,         queue_size=10)
        rospy.Subscriber            (self.target_pose_topic,            PoseStamped,    self.target_cb,     queue_size=10)
        # —— 发布器 —— #
        self.pub = rospy.Publisher  (self.Interpolation_Point_topic,    PoseStamped,                        queue_size=50)

        # --- 状态 ---
        self._lock = threading.Lock()
        self._start_T = None  # type: SE3 | None
        self._goal_T  = None  # type: SE3 | None
        self._planning = False

        rospy.loginfo("LineTrajectoryNode ready. start:=%s, target:=%s -> output:=%s",
                      self.start_pose_topic, self.target_pose_topic, self.Interpolation_Point_topic)

    # ====== 回调：起点 ======
    def ee_cb(self, msg: PoseStamped):
        '''
        功能: 使用 _lock self._pose_to_se3(msg) 将 Transformation_Matrix 赋值于 _start_T
        '''
        T = self._pose_to_se3(msg)
        with self._lock:
            self._start_T = T
        rospy.loginfo("Received start_pose @ %s", msg.header.frame_id or self.frame_id)
        # 调用 _try_plan(self)
        self._try_plan()

    # ====== 回调：目标 ======
    def target_cb(self, msg: PoseStamped):
        '''
        功能: 使用 _lock self._pose_to_se3(msg) 将 Transformation_Matrix 赋值于 _goal_T
        '''
        T = self._pose_to_se3(msg)
        with self._lock:
            self._goal_T = T
        rospy.loginfo("Received target_pose @ %s", msg.header.frame_id or self.frame_id)
        # 调用 _try_plan(self)
        self._try_plan()

    # ====== 若 start/goal 俱备则规划并发布 ======
    def _try_plan(self):
        '''
        功能: 
            检查是否已经接收到起点(start_T)和目标(goal_T)的位姿,使用互斥锁保证 _planning 安全
            满足以上情况,调用 _line_interpolate_se3(traj) 开始规划直线路径
        '''
        with self._lock:
            if self._planning:
                return
            if self._start_T is None or self._goal_T is None:
                return
            start_T = self._start_T
            goal_T  = self._goal_T
            self._planning = True

        try:
            # 直线规划路径
            traj = self._line_interpolate_se3(start_T, goal_T)
            self._publish_trajectory(traj)
        except Exception as e:
            rospy.logerr("Planning/Publishing failed: %s", e)
        finally:
            with self._lock:
                self._planning = False
                # 一次性任务：用完就清空，避免重复触发
                self._start_T = None
                self._goal_T  = None

    # ====== PoseStamped -> Pin.SE3 ======
    def _pose_to_se3(self, msg: PoseStamped) -> SE3:
        '''
        功能: 将 ROS 的 PoseStamped 消息转换为 Pinocchio 的 SE3 对象 (Transformation_Matrix)
        '''
        p = msg.pose.position
        q = msg.pose.orientation
        # 注意四元数顺序：scipy 使用 [x, y, z, w]
        quat_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)
        # 归一化以防输入不规范
        n = np.linalg.norm(quat_xyzw)
        if n == 0:
            raise ValueError("Zero-norm quaternion")
        quat_xyzw /= n
        R_mat = R.from_quat(quat_xyzw).as_matrix()
        t = np.array([p.x, p.y, p.z], dtype=float)
        return SE3(R_mat, t)

    # ====== 直线插补（位置线性 + 姿态 SLERP）======
    def _line_interpolate_se3(self, start: SE3, goal: SE3):
        '''
        功能: 在 SE3 空间内进行直线插补，生成从起点到目标的轨迹点序列。
        输入:
            start (SE3) - 起点位姿 (旋转 + 平移)
            goal  (SE3) - 目标位姿 (旋转 + 平移)

        输出:
            traj (list[SE3]) - 插值得到的位姿序列 (包含 steps 个点)
        '''
        # 计算步数
        # 如果参数 fixed_steps > 1，则强制使用固定步数
        if self.fixed_steps and self.fixed_steps > 1:
            steps = int(self.fixed_steps)
        else:
            # 否则根据两点的直线距离和 cart_res(笛卡尔分辨率) 动态计算步数
            dist = np.linalg.norm(goal.translation - start.translation)
            # 从长度上截点
            steps = max(self.min_steps, int(np.ceil(dist / max(self.cart_res, 1e-6))) + 1)

        # 从空间范围内截点
        pos_list = np.linspace(start.translation, goal.translation, steps)

        # 姿态 SLERP
        q_start = R.from_matrix(start.rotation)
        q_goal  = R.from_matrix(goal.rotation)
        slerp = Slerp([0.0, 1.0], R.concatenate([q_start, q_goal]))
        rot_list = slerp(np.linspace(0.0, 1.0, steps))

        # 组装为 SE3 序列
        traj = [SE3(Rm.as_matrix(), p) for Rm, p in zip(rot_list, pos_list)]
        rospy.loginfo("Planned %d waypoints (dist=%.3f m)", steps, np.linalg.norm(pos_list[-1]-pos_list[0]))
        return traj

    # ====== 发布轨迹点 ======
    def _publish_trajectory(self, traj):
        '''
        功能:
            将插补生成的轨迹点序列逐点发布到 ROS 话题 (PoseStamped 类型)，
            控制下游模块按设定频率依次接收目标位姿。

        输入:
            traj (list[SE3]) - 插补后的位姿序列 (包含平移 + 旋转)

        处理流程:
            1. 创建 PoseStamped 消息对象
            2. 填充 header (时间戳 + frame_id)
            3. 填充位置 (x, y, z)
            4. 填充姿态 (旋转矩阵 → 四元数 [x,y,z,w])
            5. 选取关键点 (起点/终点/每隔10%) 打印调试信息 (位置+姿态)
            6. 发布消息，并按 publish_rate_hz 控制发送频率

        输出:
            无显式返回值，通过 self.pub 发布 PoseStamped 消息
        '''
        rate = rospy.Rate(self.publish_rate_hz)
        for i, T in enumerate(traj):
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id

            # 位置
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = T.translation.tolist()

            # 姿态（转四元数 xyzw）
            quat_xyzw = R.from_matrix(T.rotation).as_quat()
            msg.pose.orientation.x = float(quat_xyzw[0])
            msg.pose.orientation.y = float(quat_xyzw[1])
            msg.pose.orientation.z = float(quat_xyzw[2])
            msg.pose.orientation.w = float(quat_xyzw[3])

            # 打印中间过程点
            rpy_deg = R.from_quat(quat_xyzw).as_euler('xyz', degrees=True)
            # rospy.loginfo("traj[%d/%d] p=(%.3f,%.3f,%.3f) rpy=(%.1f,%.1f,%.1f)deg",
            #                 i, len(traj)-1, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            #                 rpy_deg[0], rpy_deg[1], rpy_deg[2])
                

            # 通过 pub 关联到 self.pub = rospy.Publisher
            self.pub.publish(msg)
            rate.sleep()

def main():
    rospy.init_node("line_trajectory_node")
    LineTrajectoryNode()
    rospy.spin()

if __name__ == "__main__":
    main()
