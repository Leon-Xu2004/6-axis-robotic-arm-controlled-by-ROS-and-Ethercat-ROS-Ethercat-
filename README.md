# eRob3_ws 项目说明
- 版本:v2.1.1
    - 工程是一个完整的 ROS 工作空间，集中实现 eRob3 机械臂的通信、运动学求解和轨迹规划；相较于上版本，轨迹规划部分由 OMPL 代替。
    - 这个版本我主要更新了 `trajectory_node.py` ，在这里我是使用了 RRTCONNECT 算法；相较于之前进行直线插补和姿态SLERP，RRTCONNECT感觉会好一些，但是需要同样的目标点进行比对
        - 并且，我确实也需要熟悉一下代码，OMPL说实话我完全不会，全是AI写的代码，让我心慌慌
        - 我这里是在关节空间内进行路径规划，条件如下：
        | 组件        | 关节空间                             |
        | ----------- | -------------------------------- |
        | **状态空间**  | `RealVectorStateSpace(n)`        |
        | **起终状态**  | `q_start / q_goal`（由IK或已知关节位姿得到） |
        | **有效性检查** | 对 q：FK→所有link位姿→**碰撞检测**+关节限位    |
        | **采样搜索**  | RRTConnect / BIT* / PRM / KPIECE |
        | **保持连续性** | 直接在 q 空间连续；易平滑                   |

## 主要功能
- `robot_bridge` 节点桥接:
    - `eRob3_Ecat_TCP` 
        - 负责通过 TCP 与下位机通讯,接收脉冲反馈并发布 `/joint_states`,同时订阅 `/ik_joint_targets` 将轨迹点转换为带限幅的脉冲增量批量下发；
    - `YS_H7Multi_command` 
        - 将关节空间角度转换为带原点偏置的串口指令；(弃)
    - `YS_H7Multi_usart` 
        - 将这些指令写入 STM32 串口设备。(弃)
- `robot_bringup` 总launch:
    - `launch/robot_bringup.launch`
        - 集成启动 `robot_bridge`、`robot_description` 与 `robot_controll`, 一条指令完成通信、建模、控制节点的挂载。
- `robot_controll` 主控系统:
    - `fk_node.py`
        - 监听 `/joint_states`, 调用 Pinocchio 正运动学推算末端位姿并发布 `/ee_pose`;
    - `ik_node.py`
        - 订阅 `/Interpolation_Point` 与 `/joint_states`, 使用阻尼最小二乘法求解关节角, 输出 `/ik_joint_targets`;
    - `trajectory_node.py`
        - 版本 v2.0.0 ：接收 `/start_pose` 与 `/target_pose`, 进行直线插补和姿态 SLERP, 按设定频率发布 `/Interpolation_Point`;
        - 版本 v2.1.0 ：接收 `/start_pose` 与 `/target_pose`，使用 OMPL 的 RRTCONNECT 算法，
            基本上代码都是 AI 写的，这确实不太好，相较于上个版本的话，这个版本的轨迹更加丝滑和自然。
    - `main_node.py`
        - 将实时末端姿态与外部目标配对, 同步触发一次起点/目标发布, 驱动轨迹与 IK 流程衔接。
- `robot_description` 描述包:
    - `eepath_node.py` 
        - 用于订阅末端位姿 `/ee_pose`,持续累积形成 `/ee_path` 路径消息,辅助观察末端执行路径。
    - `urdf/erobot3.urdf`
        - 提供关节/连杆模型与惯性信息, 配合 `robot_state_publisher` 与 RViz 展示机械臂状态。
    - `launch/display.launch`
        - 自动加载 URDF、启动状态发布器与 RViz, 并附带末端路径可视化节点。

## BUG日志
------------------------------------------------------------------------

2025/10/8   BUG1
- The specified base path "/home/leon-xu/eRob3_ws" contains a CMakeLists.txt but "catkin_make" must be invoked in the root of workspace
    - eRob3_ws 文件夹是从另一个用户(xlh)拷贝过来的,所以它的文件所有权(owner) 还属于原来的用户。
    
------------------------------------------------------------------------


## 版本更新日志
------------------------------------------------------------------------

2025/10/8
- 版本:v1.0.0
    - 该工程在Ubuntu系统重装中受损,仅可以接受`/ik_joint_targets` 控制关节运动
    - 仍有 BUG 见 BUG1
    - 已上线 github

------------------------------------------------------------------------

2025/10/8
- 版本:v1.0.1
    - 该工程在Ubuntu系统重装中受损,仅可以接受`/ik_joint_targets` 控制关节运动
    - 解决 BUG1, 未上线 github

------------------------------------------------------------------------

2025/10/8
- 版本:v2.0.0
    - 工程是一个完整的 ROS 工作空间，集中实现 eRob3 机械臂的通信、运动学求解和轨迹规划。
    - 上线 github

------------------------------------------------------------------------
