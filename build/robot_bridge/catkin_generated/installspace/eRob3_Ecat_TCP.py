#!/usr/bin/env python3
import time, threading, socket, select, math, rospy
from sensor_msgs.msg import JointState
from collections import deque

PPR = 524288
period = 0.01
NUM_AXES = 6
JOINT_NAMES = [f"Joint{i}" for i in range(1, 7)]

def pulse_to_pi(pulse):
    return (pulse / float(PPR)) * (2.0 * math.pi)

def pi_to_pulse(angle_rad):
    return int(round((angle_rad / (2.0 * math.pi)) * PPR))

def process_data(line, joint_state):
    try:
        parts = line.strip().split()
        if len(parts) != 2: return joint_state
        axis_str, pulse_str = parts
        if not axis_str.startswith("M"): return joint_state
        axis = int(axis_str[1:]) - 1
        pulses = int(pulse_str)
        angle_rad = ((pulses % PPR) / float(PPR)) * 2 * math.pi
        if angle_rad > math.pi: angle_rad -= 2 * math.pi
        if 0 <= axis < len(joint_state.position):
            joint_state.position[axis] = angle_rad
    except Exception as e:
        rospy.logwarn("解析数据失败: %s", e)
    return joint_state


def receiver_thread(sock, pub, stop_event, current_angles, have_actual_once, lock):
    joint_state = JointState()
    joint_state.name = JOINT_NAMES
    joint_state.position = [0.0] * NUM_AXES
    buf = b""
    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            r, _, _ = select.select([sock], [], [], 0.05)
            if not r: continue
            data = sock.recv(4096)
            if not data:
                rospy.loginfo("TCP 连接关闭（接收）"); break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode().strip()
                joint_state = process_data(line, joint_state)
                joint_state.header.stamp = rospy.Time.now()
                # 同步共享角度
                with lock:
                    current_angles[:] = joint_state.position[:]
                    have_actual_once.set()
                pub.publish(joint_state)
    except Exception as e:
        rospy.logwarn("接收线程异常：%s", e)
    finally:
        rospy.loginfo("接收线程退出")


def sender_thread(sock, stop_event, current_angles, have_actual_once, lock):
    desired_hold = None
    max_dp = rospy.get_param("~max_dp", [50] * NUM_AXES)
    traj_queue = deque()
    last_cmd = [0.0] * NUM_AXES
    last_cmd_initialized = False

    def ik_target_cb(msg: JointState):
        arr = [0.0] * NUM_AXES
        n = min(NUM_AXES, len(msg.position))
        for i in range(n): arr[i] = float(msg.position[i])
        with lock:
            traj_queue.append(arr)

    rospy.Subscriber("/motion_planning/q_goal", JointState, ik_target_cb)

    next_t = time.perf_counter()
    try:
        while not rospy.is_shutdown() and not stop_event.is_set():
            with lock:
                if not last_cmd_initialized and have_actual_once.is_set():
                    last_cmd[:] = current_angles[:]
                    last_cmd_initialized = True
                while len(traj_queue) > 1:
                    traj_queue.popleft()
                if traj_queue:
                    desired_hold = traj_queue.popleft()
                desired_next = desired_hold
            pulses = [0]*NUM_AXES
            if last_cmd_initialized and desired_next is not None:
                for i in range(NUM_AXES):
                    diff_rad = desired_next[i] - last_cmd[i]
                    dp = pi_to_pulse(diff_rad)
                    lim = max_dp[i] if i < len(max_dp) else 0
                    if lim>0:
                        dp = max(-lim, min(lim, dp))
                    pulses[i] = dp
                    last_cmd[i] += pulse_to_pi(dp)
            payload_str = "CSPPACK " + " ".join(str(v) for v in pulses) + "\n"
            sock.sendall(payload_str.encode())

            next_t += period
            dt = next_t - time.perf_counter()
            if dt>0: time.sleep(dt)
            else: next_t = time.perf_counter()
    except Exception as e:
        rospy.logwarn("发送线程异常：%s", e)
    finally:
        rospy.loginfo("发送线程退出")


def main():
    rospy.init_node("eRob3_Ecat_TCP")
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    HOST, PORT = "127.0.0.1", 8080
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect((HOST, PORT))
    s.setblocking(False)
    rospy.loginfo("已连接服务器 %s:%d", HOST, PORT)

    stop_event = threading.Event()
    current_angles = [0.0]*NUM_AXES
    have_actual_once = threading.Event()
    lock = threading.Lock()

    t_rx = threading.Thread(target=receiver_thread, args=(s,pub,stop_event,current_angles,have_actual_once,lock), daemon=True)
    t_tx = threading.Thread(target=sender_thread, args=(s,stop_event,current_angles,have_actual_once,lock), daemon=True)
    t_rx.start(); t_tx.start()

    try: rospy.spin()
    finally:
        stop_event.set()
        try: s.shutdown(socket.SHUT_RDWR)
        except Exception: pass
        s.close()
        t_tx.join(timeout=1.0); t_rx.join(timeout=1.0)
        rospy.loginfo("节点正常退出")

if __name__ == "__main__":
    main()
