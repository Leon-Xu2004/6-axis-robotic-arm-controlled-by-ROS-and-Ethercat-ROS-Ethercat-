# eRob3_ws（v2.1.1）

完整的 ROS catkin 工作空间，用于 eRob3 六自由度机械臂的通讯、运动学求解与关节空间轨迹规划。本版将路径规划核心迁移至 OMPL，默认采用 RRTConnect 并带有基于 FCL 的碰撞检测。

---

- [快速开始](#快速开始)
- [目录结构](#目录结构)
- [运行时依赖](#运行时依赖)
- [核心包与节点](#核心包与节点)
- [关节空间规划管线](#关节空间规划管线)
- [RViz 与可视化](#rviz-与可视化)
- [常见问题](#常见问题)
- [版本记录](#版本记录)
- [开发提示](#开发提示)

---

## 快速开始

```bash
# 1. 进入工作空间根目录（必须在此处运行 catkin_make）
cd /home/leon-xu/eRob3_ws

# 2. 构建
catkin_make

# 3. 载入环境
source devel/setup.bash

# 4. 启动整套节点（通讯 + 运动学 + 规划 + RViz）
roslaunch robot_bringup robot_bringup.launch
```

若只想在仿真中测试规划，可省略 `robot_bridge`，单独运行：

```bash
roslaunch robot_description display.launch   # 模型与路径可视化
roslaunch robot_controll robot_controll.launch
```

## 目录结构

| 路径/包 | 说明 | 关键节点/文件 |
| --- | --- | --- |
| `robot_bridge` | 上位机与下位机的 TCP 桥接，发布反馈关节角并按脉冲增量下发控制。 | `src/eRob3_Ecat_TCP.py` |
| `robot_bringup` | 一键启动通信、模型和控制相关节点。 | `launch/robot_bringup.launch` |
| `robot_controll` | 运动学与规划模块（FK、IK、OMPL 规划、交互 marker）。 | `src/fk_node.py` / `src/ik_node.py` / `src/motion_node.py` / `src/target_marker_node.py` |
| `robot_description` | 机械臂 URDF、RViz 配置与末端路径可视化。 | `urdf/erobot3.urdf` / `rviz/erobot3.rviz` / `src/eepath_node.py` |
| `eRobot3_moveit_config` | MoveIt! 配置（目前未在默认 launch 中启用，可按需导入）。 | `launch/demo.launch` |

## 运行时依赖

| 类型 | 依赖 |
| --- | --- |
| ROS | 推荐 Ubuntu 20.04 + ROS Noetic；需要 `rospy`、`sensor_msgs`、`geometry_msgs`、`robot_state_publisher`、`rviz`、`interactive_markers` 等标准包。 |
| Python 第三方 | `numpy`、`pinocchio`、`ompl`、`python-fcl`、`trimesh`、`urdfpy`、`lxml`、`scipy`（可选，用于工作空间估计）、`matplotlib`（可选）。 |
| 系统库 | FCL 依赖的 `libfcl` 与 `octomap`；可通过 `sudo apt install ros-noetic-ompl ros-noetic-moveit ros-noetic-octomap ros-noetic-fcl` 安装。 |

> 提示：`python-fcl` 与 `pinocchio` 建议使用系统预编译版本或 `pip` 安装的二进制轮子，以避免编译耗时。

## 核心包与节点

### robot_bridge
- **`eRob3_Ecat_TCP`**：  
  - 订阅：`/motion_planning/q_goal`（JointState，规划或 IK 输出）。  
  - 发布：`/joint_states`（JointState）。  
  - 特性：使用 TCP 与下位机通信，提供脉冲限幅（`~max_dp` 参数），确保硬件安全。

### robot_controll
- **`fk_node`**：Pinocchio 正运动学，监听 `/joint_states`，输出 `/motion_kinematics/ee_pose`。  
- **`ik_node`**：阻尼最小二乘 IK，订阅 `/target_pose` 与 `/joint_states`，输出 `/motion_planning/q_goal`。  
  - 参数：`~urdf_path`、`~joint_id`（默认 6）、`~eps`、`~it_max`、`~damp` 等。  
- **`motion_node`**（v2.1.1 新）：关节空间 OMPL 规划，默认算法 RRTConnect。  
  - 输入：`/joint_states`（起点）、`/motion_planning/q_goal`（目标）。  
  - 输出：`/planned_path`（JointState 序列，可用于回放或下游处理）。  
  - 内建的 `Obstacles` 在启动时添加桌面与测试球体，并发布 MarkerArray 至 `/fcl_obstacles_array`。  
  - 关键参数：  
    | 参数 | 默认 | 用途 |
    | --- | --- | --- |
    | `~ndof` | 6 | 自由度数量 |
    | `~lower_limits`/`~upper_limits` | ±对应度数 | 关节范围（弧度） |
    | `~planner` | `RRTConnect` | 可替换为 `BITstar`, `PRM`, `KPIECE` 等 |
    | `~time_limit` | `0.5` s | 单次规划时间 |
    | `~interpolate_res` | `0.01` rad | 路径插值步长 |
    | `~publish_rate` | 60 Hz | 路径发布频率 |
- **`target_marker_node`**：在 RViz 提供 6DoF Interactive Marker，拖动即可更新 `/target_pose`。首次启动会等待 `/motion_kinematics/ee_pose` 以对齐初始位姿。
- **`main_node`**：控制 `/start_pose` 与 `/target_pose` 同步触发（默认未在 launch 中启用；按需加入）。

### robot_description
- **`robot_description.launch`**：加载 URDF，启动 `robot_state_publisher` 与 RViz。  
- **`display.launch`**：与上类似，但只用于快速可视化。  
- **`eepath_node`**：订阅 `/ee_pose` 累积成 `/ee_path`，便于在 RViz 中查看末端轨迹。  
- **`workspace.py`**：离线工具，基于随机采样估计可达工作空间并可导出点云/凸包。

## 关节空间规划管线

1. `robot_bridge/eRob3_Ecat_TCP` 每 60 Hz 发布 `/joint_states`；并在收到来自规划/IK 的关节序列时下发控制脉冲。  
2. `robot_controll/fk_node` 将 `/joint_states` 转换为末端位姿 `/motion_kinematics/ee_pose`，供 RViz 与 marker 使用。  
3. 用户在 RViz 中拖动 `target_marker_node` 提供的交互 marker，产生 `/target_pose`。  
4. `robot_controll/ik_node` 将 `/target_pose` 解算为 `/motion_planning/q_goal`。  
5. `robot_controll/motion_node` 把 `/joint_states` 作为起点、`/motion_planning/q_goal` 作为目标，在关节空间内使用 OMPL 搜索无碰撞路径，并逐点发布到 `/planned_path`。  
6. 根据使用场景，可将 `/planned_path` 回放到仿真器、或在 `robot_bridge` 中改写订阅逻辑以执行轨迹跟踪。

> 当前 `robot_controll/robot_controll.launch` 会启动 FK/IK/规划/Marker 四个节点。需要在真实硬件上运行时，再额外启动 `robot_bridge`。

### 自定义碰撞场景
- 在 `motion_node.py` 的 `main()` 中可根据需要调用 `scene.add_box` 或 `scene.add_sphere` 来添加障碍；添加后会通过 `/fcl_obstacles_array` 自动推送至 RViz。  
- `CollisionScene` 会使用 URDF 加载所有 link 的碰撞几何，并通过 FCL 计算机器人与障碍之间的碰撞/最近距离。  
- 若使用自定义 URDF，请更新 `~urdf_path` 参数或在 Launch 文件中覆盖。

## RViz 与可视化

- `roslaunch robot_description display.launch` 会加载 `erobot3.urdf` 并启动 RViz，默认配置位于 `robot_description/rviz/erobot3.rviz`。  
- `/fcl_obstacles_array`（MarkerArray）展示规划时使用的障碍物；可通过 `rostopic echo` 验证是否成功发布。  
- `/ee_path` 提供末端轨迹 Path，可在 RViz 中添加 `Path` Display 查看。  
- 若需要 MoveIt! 场景，可通过 `roslaunch eRobot3_moveit_config demo.launch` 启动（未默认启用）。

## 常见问题

- **catkin_make 提示“必须在工作空间根目录调用”**  
  - 请确保在 `/home/leon-xu/eRob3_ws` 运行 `catkin_make`，同时检查当前用户对该目录拥有读写权限（拷贝自其它用户后需 `chown -R`）。

- **RRTConnect 报 “Motion planning start tree could not be initialized!” / “Skipping invalid start state”**（2025-11-03）  
  - 含义：起始关节姿态被碰撞检测判定为无效。  
  - 排查建议：  
    1. 使用 RViz 检查桌面模型是否与基座重叠（桌面默认位于 `z=0`，可适当抬高或缩小 `scene.add_box` 尺寸）。  
    2. 确认 URDF 中每个 link 的碰撞几何都绑定到正确的 `frame_id`；若某几何落在原点，会导致误报。  
    3. 对真实硬件，确认当前关节角确实位于障碍物之外，可临时禁用某些障碍验证。  
    4. 调用 `CollisionScene.debug_distance()`（已在日志中输出）观察最近距离，判断是误判还是实际碰撞。  
    5. 若抓取物体被识别为外部障碍，可在 URDF 或规划前过滤该对象。

- **未收到 `/target_pose` / RViz marker 不出现**  
  - 确保 `target_marker_node` 与 RViz 同时运行；首次启动时会等待 `/motion_kinematics/ee_pose`，若超时则使用默认姿态。

## 版本记录

| 日期 | 版本 | 变更摘要 |
| --- | --- | --- |
| 2025-11-03 | v2.1.1 | 轨迹规划替换为 OMPL（RRTConnect），新增 FCL 场景与障碍可视化；记录起始状态碰撞问题。 |
| 2025-10-08 | v2.0.0 | ROS 工作空间重建，涵盖通讯、运动学、轨迹插值（直线 + SLERP）。 |
| 2025-10-08 | v1.0.1 | 修复 catkin_make 根目录问题，恢复 `robot_bridge` 正常构建。 |
| 2025-10-08 | v1.0.0 | 初始版本，可通过 `/ik_joint_targets` 控制，但存在构建路径问题。 |

> 旧版本中 `trajectory_node.py` 采用直线插补与姿态 SLERP。v2.1.1 已被新的 `motion_node.py` 取代。

## 开发提示

- 通过 `roslaunch robot_controll robot_controll.launch` 单独调试运动学与规划模块；在 RViz 中可实时查看 `/planned_path`、`/fcl_obstacles_array` 等话题。  
- 推荐使用 `rosrun rqt_graph rqt_graph` 理解话题流向，配合 `rostopic echo` 检查接口数据。  
- 若需要切换 OMPL 算法，可在 `motion_node` 对应的 Launch/参数服务器上修改 `~planner` 字符串。  
- `robot_description/src/workspace.py` 可用于离线生成机械臂可达空间点云，辅助规划环境设计。

