#!/usr/bin/env python3
import time
import threading
import socket
import select
import math
import rospy
from sensor_msgs.msg import JointState

# ================= 参数定义 =================
PPR = 524288   # 电机每圈脉冲数 (Pulse Per Revolution)
JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]

# ================= 数据处理函数 =================
def process_data(line, joint_state):
    """
    功能：解析一行 TCP 收到的数据，并更新关节角度
    输入：line = "M1 3294227"
    输出：更新后的 joint_state
    """
    try:
        parts = line.strip().split()
        if len(parts) != 2:
            return joint_state  # 无效数据

        axis_str, pulse_str = parts
        if not axis_str.startswith("M"):
            return joint_state

        axis = int(axis_str[1:]) - 1
        pulses = int(pulse_str)

        # 脉冲 -> 弧度，映射到 (-π, π]
        angle_rad = ((pulses % PPR) / float(PPR)) * 2 * math.pi  # [0, 2π)
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi  # (-π, π]

        if 0 <= axis < len(joint_state.position):
            joint_state.position[axis] = angle_rad

    except Exception as e:
        rospy.logwarn("解析数据失败: %s", e)

    return joint_state

# ================= 线程：发送 =================
def sender_thread(sock, stop_event):
    """
    仅负责往 TCP 发送控制指令。
    这里保留你原来的示例发送逻辑（只启用M2），并拆到专门线程中。
    需要什么就改这里或做成订阅回调写入队列（先进阶版）。
    """
    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            # === 你的原始发送片段（提到要启用的那条） ===
            sock.sendall(b"CSPMODE M1 -20\n");
            time.sleep(0.01)
            sock.sendall(b"CSPMODE M2 20\n")
            time.sleep(0.01)
            sock.sendall(b"CSPMODE M3 20\n");
            time.sleep(0.01)
            sock.sendall(b"CSPMODE M4 20\n");
            time.sleep(0.01)
            sock.sendall(b"CSPMODE M5 50\n");
            time.sleep(0.01)
            sock.sendall(b"CSPMODE M6 50\n");
            time.sleep(0.01)
    except (BrokenPipeError, OSError) as e:
        rospy.logwarn("发送线程结束（连接异常）：%s", e)
    except Exception as e:
        rospy.logwarn("发送线程异常：%s", e)
    finally:
        rospy.loginfo("发送线程退出")

# ================= 线程：接收 =================
def receiver_thread(sock, pub, stop_event):
    """
    仅负责从 TCP 读取数据、解析，并发布 /joint_states。
    """
    joint_state = JointState()
    joint_state.name = JOINT_NAMES
    joint_state.position = [0.0] * len(JOINT_NAMES)
    buf = b""

    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            # select 等待读事件
            r, _, _ = select.select([sock], [], [], 0.05)
            if not r:
                continue

            data = sock.recv(4096)
            if not data:
                rospy.loginfo("TCP 连接关闭（接收）")
                break

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode().strip()

                joint_state = process_data(line, joint_state)
                joint_state.header.stamp = rospy.Time.now()
                pub.publish(joint_state)

                rospy.loginfo("收到: %s -> %s", line, joint_state.position)

    except BlockingIOError:
        pass
    except ConnectionResetError:
        rospy.logwarn("接收线程：连接被对端重置")
    except Exception as e:
        rospy.logwarn("接收线程异常：%s", e)
    finally:
        rospy.loginfo("接收线程退出")

# ================= 主函数 =================
def main():
    rospy.init_node("Linux_tcp")

    # Publisher
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # ---------- 建立 TCP 连接 ----------
    HOST, PORT = "127.0.0.1", 8080
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect((HOST, PORT))
    s.setblocking(False)
    rospy.loginfo("已连接服务器 %s:%d", HOST, PORT)

    # ---------- 启动线程 ----------
    stop_event = threading.Event()
    t_tx = threading.Thread(target=sender_thread, args=(s, stop_event), name="tcp_sender", daemon=True)
    t_rx = threading.Thread(target=receiver_thread, args=(s, pub, stop_event), name="tcp_receiver", daemon=True)
    t_tx.start()
    t_rx.start()

    # 主线程只负责等待 ROS 停止信号
    try:
        rospy.spin()
    finally:
        # 优雅退出
        stop_event.set()
        try:
            s.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        s.close()
        t_tx.join(timeout=1.0)
        t_rx.join(timeout=1.0)
        rospy.loginfo("节点正常退出")

if __name__ == "__main__":
    main()
