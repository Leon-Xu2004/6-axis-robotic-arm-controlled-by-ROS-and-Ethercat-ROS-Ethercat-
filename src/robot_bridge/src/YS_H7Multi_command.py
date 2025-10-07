#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# 常量定义
PULSES_PER_REV = 524288
AXIS_ORIGINS = [3280000, 170000, 3250000, 3118000, 3305000, 3330000]

# 发布串口指令的话题
cmd_pub = None

def jointstate_callback(msg: JointState):
    if len(msg.position) < 6:
        rospy.logwarn("JointState does not contain enough joint positions.")
        return

    for i in range(6):
        angle_rad = msg.position[i]
        pulse = int(angle_rad * PULSES_PER_REV / (2 * 3.1415926)) + AXIS_ORIGINS[i]
        cmd = f"M{i+1} GOTO {pulse} V 30 A 1"
        cmd_pub.publish(String(data=cmd))
        rospy.loginfo(f"Sent: {cmd}")
        rospy.sleep(0.01)  # 发送间隔，确保STM32能处理完一条再下一条

def main():
    global cmd_pub
    rospy.init_node("joint_to_serial_node")
    cmd_pub = rospy.Publisher("/stm32_cmd", String, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, jointstate_callback)
    rospy.loginfo("joint_to_serial_node started, waiting for joint_states...")
    rospy.spin()

if __name__ == "__main__":
    main()
