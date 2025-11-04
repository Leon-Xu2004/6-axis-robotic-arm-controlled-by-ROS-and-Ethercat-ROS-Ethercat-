#!/usr/bin/env python3

import os
import math
import random
import threading
import numpy as np
import rospy
import trimesh
import fcl
import pinocchio as pin
from urdfpy import URDF
from lxml import etree
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from ompl import base as ob
from ompl import geometric as og

# 参数设置
class PlannerConfig:
    """
    功能说明：
    - 管理运动规划节点订阅话题以及规划参数
        - 机械臂基本参数
        - 订阅话题名
        - 路径规划参数
    """
    def __init__(self):
        # 基本
        self.ndof = rospy.get_param("~ndof", 6)                     # 机械臂自由度
        deg_limits = [360, 175, 175, 175, 360, 360]                 # 机械臂关节活动范围
        self.lower_limits = [-math.radians(x) for x in deg_limits]  # 机械臂关节活动下限
        self.upper_limits = [ math.radians(x) for x in deg_limits]  # 机械臂关节活动上限
        self.frame_id = rospy.get_param("~frame_id", "base_link")   # 机械臂底座

        # 话题
        self.q_start_topic = rospy.get_param("~q_start_topic", "/joint_states")             # 始关节角消息话题
        self.q_goal_topic  = rospy.get_param("~q_goal_topic", "/motion_planning/q_goal")    # 目标关节消息话题
        self.planned_path_topic = rospy.get_param("~planned_path_topic", "/planned_path")   # 规划结果发布话题
        self.obstacle_marker_topic = rospy.get_param("~obstacle_marker_topic", "/fcl_obstacles_array")  # RViz 可视化话题(MarkerArray)

        # 规划
        self.default_planner = rospy.get_param("~planner", "RRTConnect")        # 默认使用的 OMPL 规划算法
        self.time_limit = rospy.get_param("~time_limit", 0.5)                   # 最大规划时间限制
        self.interpolate_step = rospy.get_param("~interpolate_res", 0.01)       # 路径插值分辨率
        self.publish_rate = rospy.get_param("~publish_rate", 60)                # 轨迹发布频率

        # 模型路径
        self.urdf_path = rospy.get_param(
            "~urdf_path",
            "/home/leon-xu/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
        )
    pass

# 添加障碍模型
class Obstacles:
    """
    功能：
    - 管理场景中所有障碍物;
    - 支持添加立方体、球体和随机球体;
    - 自动发布 MarkerArray 到 RViz;
    - 提供 FCL 碰撞对象生成接口(供碰撞检测模块使用)。
    """

    def __init__(self, base_frame="base_link", topic="/fcl_obstacles_array"):
        """
        参数：
          base_frame : 所有障碍物的参考坐标系(通常为机械臂 base_link)
          topic      : RViz 可视化 MarkerArray 发布话题
        """
        self.base_frame = base_frame
        self._markers = MarkerArray()
        
        # RViz Marker 发布器(latch=True 表示 RViz 启动后仍显示)
        self.pub = rospy.Publisher(topic, MarkerArray, queue_size=10, latch=True)
        rospy.loginfo(f"[Obstacles] Ready. Base frame = {base_frame}, topic = {topic}")

    def _publish(self):
        """发布当前所有障碍物 MarkerArray 到 RViz，并打印调试信息"""
        now = rospy.Time.now()
        for m in self._markers.markers:
            m.header.stamp = now
        self.pub.publish(self._markers)

        # 打印障碍物数量
        rospy.loginfo(f"[Obstacles] Published {len(self._markers.markers)} markers to RViz.")
        # 详细打印每个障碍的参数
        for i, m in enumerate(self._markers.markers):
            if m.type == Marker.CUBE:
                rospy.loginfo(
                    f"  [Box #{i}] pos=({m.pose.position.x:.3f}, {m.pose.position.y:.3f}, {m.pose.position.z:.3f}) "
                    f"size=({m.scale.x:.3f}, {m.scale.y:.3f}, {m.scale.z:.3f}) "
                    f"color=({m.color.r:.2f}, {m.color.g:.2f}, {m.color.b:.2f}, a={m.color.a:.2f})"
                )
            elif m.type == Marker.SPHERE:
                rospy.loginfo(
                    f"  [Sphere #{i}] pos=({m.pose.position.x:.3f}, {m.pose.position.y:.3f}, {m.pose.position.z:.3f}) "
                    f"r={m.scale.x/2.0:.3f} "
                    f"color=({m.color.r:.2f}, {m.color.g:.2f}, {m.color.b:.2f}, a={m.color.a:.2f})"
                )

    def add_box(self, xyz, size, color=(1.0, 0.0, 0.0, 1.0)):
        """
        添加一个立方体障碍物并立即发布。
        参数：
          xyz   : 中心位置 (x, y, z)
          size  : 尺寸 (sx, sy, sz)
          color : RGBA 颜色
        """
        sx, sy, sz = size
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "obstacles"
        marker.id = len(self._markers.markers)
        marker.type = Marker.CUBE
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0)
        self._markers.markers.append(marker)
        self._publish()  # 每次添加后立即发布更新

    def add_sphere(self, xyz, radius, color=(0.2, 0.5, 1.0, 0.6)):
        """
        添加一个球形障碍物并立即发布。
        参数：
          xyz    : 球心位置
          radius : 半径
          color  : RGBA 颜色
        """
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.ns = "obstacles"
        marker.id = len(self._markers.markers)
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime = rospy.Duration(0)
        self._markers.markers.append(marker)
        self._publish()  # 每次添加后立即发布更新

    def add_random_spheres(self, n=10):
        """
        随机生成 n 个球形障碍物(自动避开机械臂基座区域)。
        """
        x_range, y_range, z_range = (-0.4, 0.6), (-0.4, 0.4), (0.05, 0.55)
        min_r, max_r = 0.03, 0.06
        count = 0
        for _ in range(n * 2):
            if count >= n: break
            x, y, z = random.uniform(*x_range), random.uniform(*y_range), random.uniform(*z_range)
            if (x**2 + y**2) < 0.05**2:
                continue
            r = random.uniform(min_r, max_r)
            color = (random.random(), random.random(), random.random(), 0.7)
            self.add_sphere(f"sphere_{count}", (x, y, z), r, color)
            count += 1

    def to_fcl_objects(self):
        """
        将当前 MarkerArray 转为 FCL 碰撞对象列表(用于 FCL 检测)。
        """
        objs = []
        for m in self._markers.markers:

            # 立方体障碍物 (Marker.CUBE)
            if m.type == Marker.CUBE:
                # 创建 FCL 的立方体几何体对象，尺寸与 RViz marker 一致
                geom = fcl.Box(m.scale.x, m.scale.y, m.scale.z)
                # 构造该物体在世界坐标系下的变换(平移 + 旋转)
                T = fcl.Transform(
                    np.eye(3), 
                    np.array([m.pose.position.x, 
                              m.pose.position.y, 
                              m.pose.position.z])
                )

            # 球体障碍物 (Marker.SPHERE)
            elif m.type == Marker.SPHERE:
                geom = fcl.Sphere(m.scale.x / 2.0)
                T = fcl.Transform(
                    np.eye(3), 
                    np.array([m.pose.position.x, 
                              m.pose.position.y, 
                              m.pose.position.z])
                )            
            else:
                continue
            # 将几何体 + 位姿组合成 FCL 碰撞对象，并加入列表
            objs.append(fcl.CollisionObject(geom, T))
        return objs

# 机器人碰撞模型
class CollisionScene:
    """
    统一的碰撞场景类(合并 RobotCollisionModel + CollisionWorld)：
      - 解析 URDF、加载 link 网格 → 构造 FCL 机器人几何
      - 保存障碍物(通过 Obstacles 提供的 MarkerArray 转 FCL)
      - 维护关节限位
      - 提供给规划器的 is_state_valid(q) 回调
      - 提供 debug_distance() 查询最近距离
    使用方式：
      scene = CollisionScene(urdf_path, obstacles, q_lower, q_upper)
      valid = scene.is_state_valid(q)   # 直接给 OMPL
    """

    # 加载并解析 URDF 文件
    def __init__(self, urdf_path: str, obstacles: Obstacles, q_lower, q_upper):
        """
        初始化整个碰撞场景。
        参数：
            urdf_path : str
                机器人 URDF 文件路径。
            obstacles : Obstacles
                已构建好的障碍物管理对象。
            q_lower, q_upper : list[float]
                各关节的上下限(弧度)。
        """
        # 将 urdf_path 生成一棵“树形结构”
        parser = etree.XMLParser(remove_comments=True)
        tree = etree.parse(urdf_path, parser)
        # 将树形结构存储至 root
        root = tree.getroot()

        # mesh → box 占位，保证 urdfpy 构造成功
        for mesh in root.findall(".//mesh"):
            # 获取当前对象的父节点 <geometry>
            parent = mesh.getparent()
            # 如果 parent 不存在，使用 box 替代
            if parent is not None:
                box = etree.Element("box")
                box.set("size", "0.001 0.001 0.001")
                parent.replace(mesh, box)

        # 从一个已存在的 XML 根节点构造 URDF 对象
        self.robot_urdf = URDF._from_xml(root, os.path.dirname(urdf_path))

        # 使用 Pinocchio 构建运动学模型(用于正向运动学计算)
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()     # 创建模型数据缓存

        # 用字典存储：link_name -> FCL CollisionObject
        self.fcl_links = {}
        # 加载每个 link 的网格并转为 FCL 碰撞体
        self._load_collision_geometry(urdf_path)

        # 注入障碍物与关节限位
        self.obstacles = obstacles
        self._fcl_obs_cache = None           # 障碍物 FCL 缓存(懒加载)
        self.q_lower = np.array(q_lower, dtype=float)
        self.q_upper = np.array(q_upper, dtype=float)

    # 内部函数：解析 mesh 路径(支持 package:// 与相对路径)
    def _resolve_mesh_path(self, raw_path, urdf_path):
        """
        将 URDF 中的 mesh 路径解析为绝对路径。
        """
        # 特判：如果路径包含完整工程前缀，截断掉
        if raw_path.startswith("/home/leon-xu/eRob3_ws/src/robot_description/urdf/"):
            raw_path = raw_path.replace("/home/leon-xu/eRob3_ws/src/robot_description/urdf/", "")

        # 如果是 package:// 形式(ROS 常用写法)
        if raw_path.startswith("package://"):
            # 举例：package://robot_description/meshes/link1.STL
            parts = raw_path.split('/')
            pkg_name = parts[2]  # 提取包名(例如 robot_description)
            pkg_base = f"/home/leon-xu/eRob3_ws/src/{pkg_name}"
            # 将 package://robot_description 替换为实际路径
            mesh_path = raw_path.replace(f"package://{pkg_name}", pkg_base)
        else:
            # 否则使用相对路径拼接 URDF 所在目录
            mesh_path = os.path.join(os.path.dirname(urdf_path), raw_path)

        # 规范化路径(去掉多余的“../”)
        return os.path.normpath(mesh_path)

    # 内部函数：加载 link 碰撞几何
    def _load_collision_geometry(self, urdf_path):
        """
        遍历 URDF 的所有 link/collision 节点，加载 STL 并构建 FCL BVH 模型。
        """
        from lxml import etree
        tree = etree.parse(urdf_path)
        root = tree.getroot()

        # 遍历每个 link 节点
        for link in root.findall(".//link"):
            link_name = link.attrib.get("name", "")
            # 在该 link 下查找 collision/geometry/mesh
            for col in link.findall(".//collision/geometry/mesh"):
                raw_path = col.attrib.get("filename", "")  # 获取 STL 文件路径
                mesh_path = self._resolve_mesh_path(raw_path, urdf_path)

                # 判断文件是否存在
                if not os.path.exists(mesh_path):
                    rospy.logwarn(f"[CollisionScene] mesh not found: {mesh_path}")
                    continue
                try:
                    # 使用 trimesh 加载 STL 文件
                    mesh = trimesh.load_mesh(mesh_path)
                except Exception as e:
                    rospy.logwarn(f"[CollisionScene] failed to load mesh {mesh_path}: {e}")
                    continue

                # 取出三角网格顶点与面索引
                vertices = np.array(mesh.vertices, dtype=np.float32)
                triangles = np.array(mesh.faces, dtype=np.int32)

                # 使用 FCL 构建层次包围体模型(BVHModel)以加速碰撞检测
                model = fcl.BVHModel()
                model.beginModel(len(vertices), len(triangles))
                model.addSubModel(vertices, triangles)
                model.endModel()

                # 把该 link 的几何模型包装成 FCL 碰撞对象
                self.fcl_links[link_name] = fcl.CollisionObject(model)
                rospy.loginfo(f"[CollisionScene] loaded link mesh: {link_name}")

    # 内部函数：更新机器人 link 的位姿
    def _update_robot_fk(self, q: np.ndarray):
        """
        输入关节角 q(弧度)，更新所有 link 的 FCL 碰撞体位姿。
        """
        # 使用 Pinocchio 计算前向运动学
        pin.forwardKinematics(self.model, self.data, q)
        # 更新 frame 的绝对位姿缓存(data.oMf)
        pin.updateFramePlacements(self.model, self.data)

        # 遍历每个 link 对应的 FCL 对象，设置其世界坐标姿态
        for link_name, obj in self.fcl_links.items():
            try:
                # 获取该 link 在 Pinocchio 中对应的 frame ID
                frame_id = self.model.getFrameId(link_name)
            except ValueError:
                # 如果 link 名不在模型中，跳过
                continue
            # 从 Pinocchio 获取该 link 的世界位姿(旋转矩阵 + 平移向量)
            oMf = self.data.oMf[frame_id]
            # 设置 FCL 对象的位姿
            obj.setTransform(fcl.Transform(oMf.rotation, oMf.translation))

    # 内部函数：获取/缓存 障碍物 FCL
    def _ensure_obs(self):
        if self._fcl_obs_cache is None:
            self._fcl_obs_cache = self.obstacles.to_fcl_objects()
        return self._fcl_obs_cache

    # 内部函数：状态有效性(供 OMPL 调用)
    def is_state_valid(self, q: np.ndarray) -> bool:
        """
        返回当前障碍物的 FCL 对象列表，如果未缓存则从 Obstacles 转换一次。
        """
        # 检查关节限位（任何一个关节超出上下限即无效）
        if not np.all((q >= self.q_lower) & (q <= self.q_upper)):
            rospy.logwarn("[CollisionScene] state exceeds joint limits.")
            return False

        # 更新机器人当前姿态（计算 FK）
        self._update_robot_fk(q)

        # 与障碍物进行碰撞检测
        req = fcl.CollisionRequest()  # FCL 请求对象（单次检测）
        obs = self._ensure_obs()      # 获取障碍物 FCL 对象列表

        # 遍历每个 link 与每个障碍物
        for _, obj in self.fcl_links.items():
            for o in obs:
                res = fcl.CollisionResult()
                if fcl.collide(obj, o, req, res):
                    rospy.logwarn("[CollisionScene] collision detected!")
                    return False  # 一旦碰撞，直接返回无效

        # 如果没有碰撞且未越界 → 状态有效
        return True

    # 外部函数：在当前姿态下的最近距离(调试用)
    def debug_distance(self) -> float:
        """
        计算机器人在当前姿态下（上次 update 的结果），
        与障碍物之间的最小距离（米）。
        用于调试、可视化安全裕度等。
        """
        req = fcl.DistanceRequest(enable_nearest_points=True)
        obs = self._ensure_obs()
        min_d = float("inf")

        # 遍历每个 link 和障碍物，获取最小距离
        for _, obj in self.fcl_links.items():
            for o in obs:
                res = fcl.DistanceResult()
                d = fcl.distance(obj, o, req, res)
                min_d = min(min_d, d)

        # 如果距离有效则返回数值，否则返回 None
        return min_d if np.isfinite(min_d) else None

# 规划器(只依赖回调)
class OmplPlanner:
    """只负责 OMPL：状态空间 + 规划，不直接访问机器人/障碍。"""
    def __init__(self, ndof, lower_limits, upper_limits, time_limit, interpolate_step, validity_cb):
        """
        初始化 OMPL 规划器。

        参数说明：
            ndof : int
                机械臂自由度数量 (Degrees of Freedom)
            lower_limits : list[float]
                每个关节的最小角度（弧度）
            upper_limits : list[float]
                每个关节的最大角度（弧度）
            time_limit : float
                规划时间上限 (秒)
            interpolate_step : float
                插值分辨率（弧度）
                决定最终路径点的密度和平滑度
            validity_cb : callable
                状态有效性检测函数
                → 输入关节角 q(np.ndarray)
                → 返回 True(可行) / False(碰撞或越界)
        """
        # 基本参数与回调函数绑定
        self.ndof = ndof                            # 自由度数量
        self.time_limit = time_limit                # 最大规划时间
        self.interpolate_step = interpolate_step    # 插值分辨率
        self.validity_cb = validity_cb              # 状态有效性函数

        # 定义 OMPL 状态空间 (Joint Space)
        self.space = ob.RealVectorStateSpace(ndof)

        # 定义状态空间边界
        bounds = ob.RealVectorBounds(ndof)
        for i in range(ndof):
            bounds.setLow(i, lower_limits[i])       # 设置第 i 维下限
            bounds.setHigh(i, upper_limits[i])      # 设置第 i 维上限
        self.space.setBounds(bounds)                # 应用边界限制

    # 内部函数：包装 validity_cb，用于 OMPL 调用
    def _is_valid(self, state):
        """
        OMPL 的 StateValidityChecker 需要一个函数：
            输入 ob.State 类型 → 返回 bool
        所以我们这里转换成 numpy 数组后再调用外部的 validity_cb。
        """
        q = np.array([state[i] for i in range(self.ndof)], dtype=float)
        # 把 OMPL 状态对象 state 转为 numpy 向量 q
        return self.validity_cb(q)
    
    # 主函数：路径规划
    def plan(self, q_start: np.ndarray, q_goal: np.ndarray):
        """
        使用 RRTConnect 算法在 joint-space 中规划路径。

        输入：
            q_start : np.ndarray
                起始关节角向量
            q_goal : np.ndarray
                目标关节角向量

        输出：
            path : ompl.geometric.PathGeometric
                规划成功 → 返回路径对象;
                规划失败 → 返回 None。
        """
        # 创建 OMPL 的 SimpleSetup
        ss = og.SimpleSetup(self.space)

        # 设置状态有效性检查函数（使用包装过的 _is_valid）
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_valid))

        # 定义起点与目标状态
        start, goal = ob.State(self.space), ob.State(self.space)
        for i in range(self.ndof):
            start[i] = float(q_start[i])
            goal[i]  = float(q_goal[i])
        # OMPL 将从 start 开始，在 joint space 中采样搜索直到到达 goal。
        ss.setStartAndGoalStates(start, goal)

        # 选择规划算法 (RRTConnect)
        planner = og.RRTConnect(ss.getSpaceInformation())
        ss.setPlanner(planner)

        # 执行规划
        if not ss.solve(self.time_limit):
            # 超时或失败 → 返回 None
            rospy.logwarn(f"[OmplPlanner] Planning failed (timeout {self.time_limit}s).")
            return None

        # 后处理：路径简化与插值平滑
        ss.simplifySolution()
        # OMPL 内置简化：Shortcut（截断冗余路径）+ B-spline 平滑

        # 获取规划结果路径对象
        path = ss.getSolutionPath()

        # 根据设定的步长插值，增加路径点数量
        # path.length() 返回路径长度（弧度单位）
        n_points = max(2, int(path.length() / self.interpolate_step))
        path.interpolate(n_points)

        rospy.loginfo(f"[OmplPlanner] ✅ Path planned with {n_points} points.")
        return path

# ROS I/O(只做接口)
class PlannerNodeInterface:
    """ 
    功能：
    - 只负责 ROS 通信部分（订阅 / 发布）;
    - 当起点和目标都收到时，调用外部规划函数;
    - 将规划出的路径逐点发布为 JointState;
    - 不参与规划算法 / 碰撞检测逻辑。
    """
    def __init__(self, ndof, q_start_topic, q_goal_topic,
                 planned_path_topic, publish_rate_hz,
                 plan_fn):
        # 保存基础配置
        self.ndof = ndof
        self.plan_fn = plan_fn
        self.publish_rate_hz = publish_rate_hz

        # 用于临时存储起点与目标姿态
        self.q_start = None
        self.q_goal = None

        # 线程锁：防止两个回调函数同时写入 q_start/q_goal
        self.lock = threading.Lock()

        # ROS 通信部分
        # 发布器：用于输出规划路径 JointState 序列
        self.pub = rospy.Publisher(planned_path_topic, JointState, queue_size=100)
        # 订阅起点话题
        rospy.Subscriber(q_start_topic, JointState, self._cb_start)
        # 订阅目标话题
        rospy.Subscriber(q_goal_topic , JointState, self._cb_goal)
        rospy.loginfo("[PlannerNodeInterface] subscribed to start & goal topics.")

    # 起点消息回调
    def _cb_start(self, msg: JointState):
        """
        当收到起点关节状态时：
          1. 把 JointState.position 转为 numpy 数组;
          2. 存入 self.q_start;
          3. 尝试触发规划（如果目标已存在）。
        """
        with self.lock:
            self.q_start = np.array(msg.position, dtype=float)
        self._try_plan()  

    # 目标消息回调
    def _cb_goal(self, msg: JointState):
        """
        当收到目标关节状态时：
          1. 把 JointState.position 转为 numpy 数组;
          2. 存入 self.q_goal;
          3. 尝试触发规划（如果起点已存在）。
        """
        with self.lock:
            self.q_goal = np.array(msg.position, dtype=float)
        self._try_plan()

    # 尝试触发规划
    def _try_plan(self):
        """
        若同时收到了起点和目标，就调用规划函数 plan_fn。
        规划完成后将结果逐点发布。
        """
        with self.lock:
            # 若任一为空，则暂不触发
            if self.q_start is None or self.q_goal is None:
                return

            # 获取并清空缓存，防止重复触发
            q_start, q_goal = self.q_start, self.q_goal
            self.q_start = None
            self.q_goal = None

        rospy.loginfo("🚀 Planning from start → goal ...")

        # 调用外部规划函数（由 main() 传入）
        # 通常对应 OmplPlanner.plan()
        path = self.plan_fn(q_start, q_goal)

        # 若规划失败（None），直接返回
        if path is None:
            rospy.logwarn("[PlannerNodeInterface] Planning failed.")
            return
        # 成功则发布路径
        self._publish_path(path)

    # 路径发布
    def _publish_path(self, path):
        """
        将 OMPL 生成的路径逐点发布为 JointState。
        用于在 RViz 或下游节点中播放规划轨迹。
        """
        msg = JointState()  # 创建 JointState 消息对象

        # 填 joint 名称，方便 RViz 匹配（joint_1 ... joint_N）
        msg.name = [f"joint_{i+1}" for i in range(self.ndof)]

        # 定义发布频率（控制播放速度）
        rate = rospy.Rate(self.publish_rate_hz)

        # 遍历 OMPL 路径中的每个状态点
        for s in path.getStates():
            msg.header.stamp = rospy.Time.now()  # 时间戳
            # 把每个维度的状态写入 position
            msg.position = [s[i] for i in range(self.ndof)]
            # 发布
            self.pub.publish(msg)
            # 等待下一次发布
            rate.sleep()

        rospy.loginfo("[PlannerNodeInterface] Path published successfully.")
    pass

# main主函数
def main():
    rospy.init_node("ompl_planner_node")

    # 装载配置
    cfg = PlannerConfig()
    # 构建障碍物
    scene = Obstacles(base_frame="base_link")
    # 添加障碍物
    scene.add_box((0.0, 0.0, -0.05), (1.0, 1.0, 0.05))          # 桌面
    scene.add_sphere((0.45, 0.0, 0.25), 0.06)                   # 球形障碍
    # scene.add_random_spheres(3)                                 # 随机障碍

    # 加载障碍传感
    Collisionworld = CollisionScene(cfg.urdf_path, scene, cfg.lower_limits, cfg.upper_limits)

    # 规划器
    planner = OmplPlanner(
        ndof=cfg.ndof,
        lower_limits=cfg.lower_limits,
        upper_limits=cfg.upper_limits,
        time_limit=cfg.time_limit,
        interpolate_step=cfg.interpolate_step,
        validity_cb=Collisionworld.is_state_valid
    )

    # 计划函数(主进程控制日志/调试)
    def plan_fn(q_start: np.ndarray, q_goal: np.ndarray):
        path = planner.plan(q_start, q_goal)
        if path is not None:
            rospy.loginfo("✅ path points: %d", len(path.getStates()))
            # 可选：最近距离调试
            d = Collisionworld.debug_distance()
            if d is not None:
                rospy.loginfo("[DebugDist] min distance to obstacles: %.4f m", d)
        return path

    # ROS I/O
    PlannerNodeInterface(
        ndof=cfg.ndof,
        q_start_topic=cfg.q_start_topic,
        q_goal_topic=cfg.q_goal_topic,
        planned_path_topic=cfg.planned_path_topic,
        publish_rate_hz=cfg.publish_rate,
        plan_fn=plan_fn
    )

    rospy.loginfo("✅ OMPL Planner Node v2.3 (decoupled) started.")
    rospy.sleep(1.0)
    scene._publish()     # <—— 关键补发
    rospy.spin()

if __name__ == "__main__":
    main()
