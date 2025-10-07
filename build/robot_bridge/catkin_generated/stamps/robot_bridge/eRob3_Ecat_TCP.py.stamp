#!/usr/bin/env python3
import time
import threading
import socket
import select
import math
import rospy
from sensor_msgs.msg import JointState
from collections import deque

PPR = 524288        # 电机每圈脉冲数 (Pulse Per Revolution)
period = 0.01      # 1ms/帧
NUM_AXES = 6
JOINT_NAMES = ["Joint1", 
               "Joint2", 
               "Joint3", 
               "Joint4", 
               "Joint5", 
               "Joint6"]


def pulse_to_pi(pulse):
    # 脉冲 → 弧度
    return (pulse / float(PPR)) * (2.0 * math.pi)

def pi_to_pulse(angle_rad):
    # 弧度 → 脉冲
    return int(round((angle_rad / (2.0 * math.pi)) * PPR))

def process_data(line, joint_state):
    """
    功能：解析一行 TCP 收到的数据，并更新关节角度
    输入：line = "M1 3294227"
    输出：更新后的 joint_state
    """
    try:

        # 先去掉行首和行尾的多余空格/换行符；
        # 再按空格把字符串拆分成若干部分。
        parts = line.strip().split()
        if len(parts) != 2:
            return joint_state  # 无效数据
        
        # 序列解包
        axis_str, pulse_str = parts
        if not axis_str.startswith("M"):
            return joint_state

        axis = int(axis_str[1:]) - 1 
        pulses = int(pulse_str) 
        # 关节角度(弧度制) = (脉冲数的取余) / PPR 后映射到 [0, 2π) 
        angle_rad = ((pulses % PPR) / float(PPR)) * 2 * math.pi 
        # 将弧度，映射到 (-π, π] 
        if angle_rad > math.pi: 
            angle_rad -= 2 * math.pi # (-π, π] 
        if 0 <= axis < len(joint_state.position): 
            joint_state.position[axis] = angle_rad 
    except Exception as e: 
        rospy.logwarn("解析数据失败: %s", e) 
    return joint_state



def receiver_thread(sock, pub, stop_event):
    """
    功能：专门负责从 TCP 套接字中读取电机状态数据，并解析后发布 ROS 话题 /joint_states。
    输入：sock : socket.socket
            已连接的 TCP 套接字，用于接收下位机发来的电机状态数据（例如 "M1 3294227"）。
        pub : rospy.Publisher
            ROS 发布者对象，用于向 /joint_states 话题发布 JointState 消息。
        stop_event : threading.Event
            线程停止信号。如果被 set()，线程会安全退出。
    输出：输出脉冲至/joint_states
    """
        # 初始化 JointState 消息对象
    joint_state = JointState()
    # 设置关节名称（与 URDF 中定义的 joint 名称一致）
    joint_state.name = JOINT_NAMES
    # 初始化所有关节位置为 0.0，长度与关节数一致
    joint_state.position = [0.0] * len(JOINT_NAMES)
    # 缓冲区，用于处理 TCP 数据的粘包/拆包问题
    buf = b""

    try:
        # 主循环：当 ROS 节点未关闭，且未收到外部停止信号时持续运行
        while not rospy.is_shutdown() and not stop_event.is_set():
            # 使用 select 监听套接字是否可读，超时 50ms
            # 这样可以避免 sock.recv() 阻塞线程
            r, _, _ = select.select([sock], [], [], 0.05)
            if not r:
                # 若无数据到达，跳过当前循环继续监听
                continue

            # 从 socket 读取最多 4096 字节数据
            data = sock.recv(4096)
            if not data:
                # 若 recv 返回空，说明对端关闭了连接
                rospy.loginfo("TCP 连接关闭（接收）")
                break

            # 将新接收的数据追加到缓冲区
            buf += data

            # 处理缓冲区中的完整行（以换行符 \n 分隔）
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode().strip()

                # —— 打印 “M轴: 弧度” ——
                parts = line.split()
                if len(parts) == 2 and parts[0].startswith("M"):
                    try:
                        axis = int(parts[0][1:])
                        pulses = int(parts[1])
                        # 脉冲 -> 弧度
                        angle_rad = ((pulses % PPR) / float(PPR)) * 2 * math.pi
                        if angle_rad > math.pi:
                            angle_rad -= 2 * math.pi
                        # rospy.loginfo("M%d: %.6f rad", axis, angle_rad)
                    except ValueError:
                        rospy.logwarn("无效行（无法解析为轴/脉冲）: %s", line)
                else:
                    rospy.logwarn("无效行（格式应为 'M# <pulses>'）: %s", line)

                # 保持原有 JointState 更新与发布
                joint_state = process_data(line, joint_state)
                joint_state.header.stamp = rospy.Time.now()
                # 发送 JointState 消息对象
                pub.publish(joint_state)


    # 异常处理
    except BlockingIOError:
        # 非阻塞模式下可能出现此异常，忽略即可
        pass
    except ConnectionResetError:
        # 如果连接被对端重置（下位机异常断开），打印警告日志
        rospy.logwarn("接收线程：连接被对端重置")
    except Exception as e:
        # 捕获其他未知异常，打印警告信息
        rospy.logwarn("接收线程异常：%s", e)
    finally:
        # 无论是否发生异常，退出时都会执行
        rospy.loginfo("接收线程退出")


def sender_thread(sock, stop_event):

    desired_hold = None   # 记住“上一次目标”，队列空时继续朝它走

    # 每周期每轴最大允许脉冲（安全限幅），可在 launch 里覆盖
    max_dp = rospy.get_param("~max_dp", [50, 50, 50, 50, 50, 50])
    if not isinstance(max_dp, list) or len(max_dp) < NUM_AXES:
        max_dp = [50] * NUM_AXES

    # 队列缓存轨迹点（弧度）
    traj_queue = deque()
    lock = threading.Lock()

    # 实际关节角（仅用于首次对齐与健康监控，不用于每步求差）
    current_angles = [0.0] * NUM_AXES
    have_actual_once = False

    # 上一次“实际下发后的内部目标”（弧度）
    last_cmd = [0.0] * NUM_AXES
    last_cmd_initialized = False

    def ik_target_cb(msg: JointState):
        """订阅 IK 轨迹目标（弧度），放入队列逐点下发"""
        with lock:
            arr = [0.0] * NUM_AXES
            n = min(NUM_AXES, len(msg.position))
            for i in range(n):
                arr[i] = float(msg.position[i])
            traj_queue.append(arr)

    def actual_cb(msg: JointState):
        """订阅实际关节角（弧度），用于首帧对齐与监控"""
        nonlocal have_actual_once
        with lock:
            n = min(NUM_AXES, len(msg.position))
            for i in range(n):
                current_angles[i] = float(msg.position[i])
            have_actual_once = True

    rospy.Subscriber("/ik_joint_targets", JointState, ik_target_cb)
    rospy.Subscriber("/joint_states", JointState, actual_cb)

    next_t = time.perf_counter()
    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            with lock:
                # 首次对齐：把 last_cmd 设为当前实际角，避免第一步大跳
                if not last_cmd_initialized and have_actual_once:
                    last_cmd = current_angles[:]  # 复制
                    last_cmd_initialized = True

                # 取一个轨迹点（若为空则本周期不动）
                # 可选：把积压的旧点丢掉，只保留最新，避免“追历史”导致顿挫
                while len(traj_queue) > 1:
                    traj_queue.popleft()

                if traj_queue:
                    desired_hold = traj_queue.popleft()
                # 队列空，就沿用上一次目标（保持匀速逼近）
                desired_next = desired_hold
            
            pulses = [0] * NUM_AXES

            if last_cmd_initialized and desired_next is not None:
                # 增量 = 轨迹下一点 - 上一次“已下发后的内部目标”
                for i in range(NUM_AXES):
                    diff_rad = desired_next[i] - last_cmd[i]
                    dp = pi_to_pulse(diff_rad)  # 弧度 -> 脉冲

                    # 安全限幅
                    lim = max_dp[i] if i < len(max_dp) else 0
                    if lim > 0:
                        if dp >  lim: dp =  lim
                        if dp < -lim: dp = -lim

                    pulses[i] = dp
                    # 用“实际下发的 Δp”折回更新 last_cmd（保持内部一致）
                    last_cmd[i] += pulse_to_pi(dp)

            # 调试输出（每秒一次）：队列长度、下发的 Δp、内部 q_cmd
            # rospy.loginfo_throttle(1.0, f"traj_len={len(traj_queue)} Δp={pulses} q_cmd={[round(a,4) for a in last_cmd]}")

            # 发送
            payload_str = "CSPPACK " + " ".join(str(v) for v in pulses) + "\n"
            sock.sendall(payload_str.encode())
            # rospy.loginfo_throttle(0.5, 
            #     f"traj_len={len(traj_queue)} "
            #     f"Δp={pulses} "
            #     f"desired_next={[round(a,4) if a is not None else None for a in (desired_next or [])]} "
            #     f"last_cmd={[round(a,4) for a in last_cmd]}"
            # )


            # 固定周期
            next_t += period
            now = time.perf_counter()
            dt = next_t - now
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = now

    except (BrokenPipeError, OSError) as e:
        rospy.logwarn("发送线程结束（连接异常）：%s", e)
    except Exception as e:
        rospy.logwarn("发送线程异常：%s", e)
    finally:
        rospy.loginfo("发送线程退出")


'''def sender_thread(sock, stop_event):
    """
    每周期发送一条批量命令：
        CSPPACK v1 v2 v3 v4 v5 v6\n
    这里 v1..v6 来自 ROS 参数 ~values 数组
    """
    period = float(rospy.get_param("~period", 0.001))  # 控制周期（秒）
    # 直接使用数组，每个轴一个数
    values = rospy.get_param("~values", [0, 0, 0, 0, 0, 0])

    if not isinstance(values, list) or len(values) < 6:
        rospy.logerr("参数 ~values 必须是包含6个元素的数组，例如 [0,0,10,0,0,0]")
        return

    next_t = time.perf_counter()
    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            # 构造批量命令字符串
            payload_str = f"CSPPACK {values[0]} {values[1]} {values[2]} {values[3]} {values[4]} {values[5]}\n"
            sock.sendall(payload_str.encode())

            # 固定周期对时
            next_t += period
            now = time.perf_counter()
            dt = next_t - now
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = now
    except (BrokenPipeError, OSError) as e:
        rospy.logwarn("发送线程结束（连接异常）：%s", e)
    except Exception as e:
        rospy.logwarn("发送线程异常：%s", e)
    finally:
        rospy.loginfo("发送线程退出")'''


def main():
    """
    主函数：初始化 ROS 节点，建立 TCP 连接，并启动发送/接收线程。

    功能流程:
        ----------
        1. 初始化 ROS 节点
        2. 创建 Publisher 发布 /joint_states
        3. 建立 TCP 连接到服务器（下位机或仿真程序）
        4. 启动发送线程和接收线程
        5. 主线程通过 rospy.spin() 等待退出信号
        6. 程序退出时优雅地关闭连接和线程
    """ 
    # 初始化 ROS 节点
    rospy.init_node("eRob3_Ecat_TCP")

    # 创建 ROS 发布者，用于发布关节状态消息
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # 建立 TCP 连接
    HOST, PORT = "127.0.0.1", 8080  # 服务器地址和端口（此处本地测试用 127.0.0.1:8080）
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建 TCP 套接字
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 禁用 Nagle 算法，降低延迟
    s.connect((HOST, PORT))  # 主动连接服务器
    s.setblocking(False)     # 设置为非阻塞模式，避免 recv/send 阻塞线程
    rospy.loginfo("已连接服务器 %s:%d", HOST, PORT)

    # 启动发送/接收线程
    stop_event = threading.Event()  # 用于线程间通信，标记是否需要停止
    # 启动发送线程：负责将 ROS 指令通过 TCP 发送给下位机
    t_tx = threading.Thread(
        target=sender_thread,
        args=(s, stop_event),
        name="tcp_sender",
        daemon=True  # 守护线程，随主线程退出而退出
    )
    # 启动接收线程：负责接收下位机反馈并发布 /joint_states
    t_rx = threading.Thread(
        target=receiver_thread,
        args=(s, pub, stop_event),
        name="tcp_receiver",
        daemon=True
    )
    t_tx.start()
    t_rx.start()

    # 主线程等待 ROS 退出
    try:
        rospy.spin()  # 主线程阻塞，直到 ROS 节点关闭（Ctrl+C 或 rosnode kill）
    finally:
        # 退出
        stop_event.set()  # 通知子线程退出
        try:
            # 尝试关闭 TCP 连接（双向）
            s.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        s.close()  # 关闭 socket

        # 等待发送/接收线程结束（最多等待 1 秒）
        t_tx.join(timeout=1.0)
        t_rx.join(timeout=1.0)

        rospy.loginfo("节点正常退出")
        

# 程序入口
if __name__ == "__main__":
    main()
