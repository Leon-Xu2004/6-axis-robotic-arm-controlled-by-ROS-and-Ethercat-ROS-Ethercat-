#!/usr/bin/env python3
import rospy, socket, select
from sensor_msgs.msg import JointState
import math

# 电机每圈脉冲数
PPR = 524288  

# 关节名称（你需要和 URDF 里的 joint 名称对齐）
JOINT_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]

# ---------------- 处理函数 ----------------
def process_data(line, joint_state):
    """
    处理一行TCP数据，例如 "M1 3294227"
    更新 joint_state 并返回
    """
    try:
        parts = line.strip().split()
        if len(parts) != 2:
            return joint_state  # 格式不对直接丢弃

        axis_str, pulse_str = parts
        if not axis_str.startswith("M"):
            return joint_state  # 非法数据

        axis = int(axis_str[1:]) - 1  # 轴号从0开始
        pulses = int(pulse_str)

        # 脉冲 → 弧度
        angle_deg = (pulses % PPR) / float(PPR) * 2 * math.pi
        angle_rad = math.radians(angle_deg)         # 再转弧度


        # 更新对应关节角度
        if 0 <= axis < len(joint_state.position):
            joint_state.position[axis] = angle_rad

    except Exception as e:
        rospy.logwarn("解析数据失败: %s", e)

    return joint_state


# ---------------- 主函数 ----------------
def main():
    rospy.init_node("Linux_tcp")

    # JointState 发布器
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # 初始化 JointState 消息
    joint_state = JointState()
    joint_state.name = JOINT_NAMES
    joint_state.position = [0.0] * len(JOINT_NAMES)

    HOST, PORT = "127.0.0.1", 8080
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.setblocking(False)

    rospy.loginfo("已连接服务器 %s:%d", HOST, PORT)

    buf = b""
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        r, _, _ = select.select([s], [], [], 0.05)
        if r:
            try:
                data = s.recv(4096)
            except BlockingIOError:
                continue
            if not data:
                rospy.loginfo("TCP 连接关闭")
                break

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.decode().strip()

                # === 调用数据处理函数 ===
                joint_state = process_data(line, joint_state)
                joint_state.header.stamp = rospy.Time.now()
                pub.publish(joint_state)

                rospy.loginfo("收到: %s -> %s", line, joint_state.position)

        rate.sleep()

    s.close()
    rospy.loginfo("节点正常退出")


if __name__ == "__main__":
    main()
