#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from ompl import base as ob
from ompl import geometric as og


class OmplPlannerNode:
    """
    V2.1.1 SE3 规划节点 (RRTConnect / KPIECE1 / EST)
      - 输入：/start_pose, /target_pose (PoseStamped)
      - 输出：/planned_path (PoseStamped 序列)
      - 仅工作空间边界有效性（无障碍/无关节限位）
    """

    # ---------- 生命周期 ----------
    def __init__(self):
        self._load_params()
        self._init_publishers_subscribers_services()
        self._init_internal_state()
        rospy.loginfo("OmplPlannerNode ready: planner=%s, time_limit=%.2fs",
                      self.default_planner, self.time_limit)

    # ---------- 参数设定 ----------
    def _load_params(self):
        """初始化话题、规划参数、工作空间、空间定义"""

        # 定义关节角度限制
        deg_limits = [
            360,  # J1
            175,  # J2
            175,  # J3
            175,  # J4
            360,  # J5
            360   # J6
        ]

        # 话题与坐标系
        self.q_start_topic = rospy.get_param("~q_start_topic", "/joint_states")
        self.q_goal_topic = rospy.get_param("~q_goal_topic", "/motion_planning/q_goal")
        self.planned_path_topic = rospy.get_param("~planned_path_topic", "/planned_path")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.ndof = rospy.get_param("~ndof", 6)
        self.lower_limits = [-math.radians(x) for x in deg_limits]
        self.upper_limits = [ math.radians(x) for x in deg_limits]

        # 规划参数
        self.default_planner  = rospy.get_param("~planner", "RRTConnect")  # RRTConnect/KPIECE1/EST
        self.time_limit       = float(rospy.get_param("~time_limit", 0.5))
        self.interpolate_step = float(rospy.get_param("~interpolate_res", 0.01))  # 估算插值点数用
        self.publish_rate_hz  = int(rospy.get_param("~publish_rate", 60))
        self.si_resolution    = float(rospy.get_param("~si_resolution", 0.01))    # 有效性检查分辨率

        # --- 定义关节空间 ---
        self.space = ob.RealVectorStateSpace(self.ndof)
        bounds = ob.RealVectorBounds(self.ndof)
        for i in range(self.ndof):
            bounds.setLow(i, float(self.lower_limits[i]))
            bounds.setHigh(i, float(self.upper_limits[i]))
        self.space.setBounds(bounds)
        self.bounds = bounds


    # ---------- ROS 通信接口 ----------
    def _init_publishers_subscribers_services(self):
        """订阅：/joint_states, /target_pose；发布：/planned_path"""
        rospy.Subscriber(self.q_start_topic, JointState, self._cb_start, queue_size=10)
        rospy.Subscriber(self.q_goal_topic, JointState, self._cb_goal, queue_size=10)
        self.pub_path = rospy.Publisher(self.planned_path_topic, JointState, queue_size=100)

    def _init_internal_state(self):
        """创建线程同步的互斥锁与输入缓存"""
        self._lock = threading.Lock()
        self._planning = False
        self.q_start = None  # PoseStamped
        self.q_goal  = None  # PoseStamped

    # ---------- 回调层(收集输入，触发规划) ----------
    def _cb_start(self, msg):
        with self._lock:
            self.q_start = np.array(msg.position, dtype=float)
        self._try_plan()
    def _cb_goal(self, msg):
        with self._lock:
            self.q_goal = np.array(msg.position, dtype=float)
        self._try_plan()

    # ---------------- 有效性检查 ----------------
    def _is_state_valid(self, state):
        """检查关节限位 + 占位FK碰撞检测"""
        q = np.array([state[i] for i in range(self.ndof)], dtype=float)
        # 1) 检查关节限位
        for i in range(self.ndof):
            if q[i] < self.lower_limits[i] or q[i] > self.upper_limits[i]:
                return False
        # 2) 预留：FK + 碰撞检测
        # fk_pose = self.forward_kinematics(q)
        # if self.check_collision(fk_pose): return False
        return True

    # ---------------- 规划流程 ----------------
    def _try_plan(self):
        """
        功能：检查是否具备起点与目标点，并启动一次规划任务
        Function: Check if both start & goal joint states are ready, then perform one planning attempt.
        """
        with self._lock:
            if self._planning:
                # rospy.logwarn("Planning already in progress, skipping new request.")
                return

            # ✅ 正确检查：起点和目标都要存在
            if self.q_start is None or self.q_goal is None:
                # rospy.loginfo("Waiting for both start and goal JointState...")
                return

            # 提取并清空缓存
            q_start = self.q_start
            q_goal  = self.q_goal
            self.q_start = None
            self.q_goal  = None
            self._planning = True

        rospy.loginfo("Start planning from start_pose → goal_pose ...")

        try:
            # 执行一次规划
            result = self._plan_once(q_start, q_goal, self.default_planner, self.time_limit)
            if not result:
                rospy.logwarn("OMPL planning failed: no path found.")
                return
            
            # 返回结果：成功标志、路径点数、状态信息
            success, point_count, status_msg = result
            if success:
                rospy.loginfo(f"✅ OMPL planning success: {point_count} points, {status_msg}")
            else:
                rospy.logwarn(f"⚠️ OMPL planning failed: {status_msg}")
        except Exception as e:
            # 捕获异常防止节点崩溃
            rospy.logerr(f"❌ Planning error: {e}")
        finally:
            # 无论成功失败都要解锁
            with self._lock:
                self._planning = False


    # ---------- 最小实现：SimpleSetup 一把梭 ----------
    def _plan_once(self, q_start, q_goal, planner_name, time_limit):
        """
        功能：使用 OMPL SimpleSetup 执行一次完整的关节空间路径规划
        Function: Perform one complete OMPL planning process in joint space via SimpleSetup.
        """
        rospy.loginfo("Planning joint path from start → goal ...")

        # 创建 SimpleSetup 实例（核心 OMPL 接口）
        ss = og.SimpleSetup(self.space)
        # 注册状态有效性检查器（用于碰撞检测）
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))

        # 获取空间信息对象，用于配置规划器
        si = ss.getSpaceInformation()
        si.setup()

        # 创建起点与目标状态（OMPL State 类型）
        start = ob.State(self.space)
        goal  = ob.State(self.space)
        for i in range(self.ndof):
            start[i] = float(q_start[i])
            goal[i]  = float(q_goal[i])

        # 设置起止状态
        ss.setStartAndGoalStates(start, goal)

        # 选择规划算法
        planner = self._select_planner(si, planner_name)
        ss.setPlanner(planner)
        
        # 执行规划，时间上限由参数控制
        solved = ss.solve(time_limit)
        if not solved:
            rospy.logwarn("No path found within %.2fs", time_limit)
            self._planning = False
            return False

        # 对路径进行简化（去冗余）
        ss.simplifySolution()
        path = ss.getSolutionPath()

        # 计算路径长度与插值数量
        length = path.length()
        n_interp = max(2, int(np.ceil(length / max(self.interpolate_step, 1e-6))))

        # 插值，使路径更平滑
        path.interpolate(n_interp)

        # 发布路径给 ROS 话题
        self._publish_path(path)
        
        rospy.loginfo("✅ Planned %d waypoints (%.3f rad)", n_interp, length)
        self._planning = False

        return True, n_interp, f"length≈{length:.3f} rad"


    # ---------- Planner 选择 ----------
    def _select_planner(self, si, name):
        """
        功能：根据输入字符串选择具体 OMPL 规划算法
        Function: Select OMPL planner instance based on string name.
        """
        n = (name or "").lower()
        if "rrt" in n and "connect" in n:
            return og.RRTConnect(si)
        elif "kpiece" in n:
            return og.KPIECE1(si)
        elif "est" in n:
            return og.EST(si)
        else:
            rospy.logwarn("Unknown planner '%s', using RRTConnect", name)
            return og.RRTConnect(si)
        
    # ---------- 输出/工具 ----------
    def _publish_path(self, path_geometric):
        """
        功能：将 OMPL 输出的路径以 JointState 格式逐点发布
        Function: Publish the planned path as a stream of JointState messages.
        """
        states = path_geometric.getStates()
        msg = JointState()
        msg.name = [f"joint_{i+1}" for i in range(self.ndof)]
        rate = rospy.Rate(self.publish_rate_hz)

        # 逐状态发布（类似播放轨迹）
        for s in states:
            msg.header.stamp = rospy.Time.now()
            msg.position = [s[i] for i in range(self.ndof)]
            self.pub_path.publish(msg)
            rate.sleep()
    pass


def main():
    rospy.init_node("ompl_planner_node")
    OmplPlannerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
