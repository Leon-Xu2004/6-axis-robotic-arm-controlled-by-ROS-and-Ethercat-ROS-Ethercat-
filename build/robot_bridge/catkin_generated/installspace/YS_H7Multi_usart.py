#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import String

# 串口对象初始化为全局变量
ser = None

# 回调函数：当接收到 /stm32_cmd 的字符串数据时调用
def write_to_stm32(msg):
    if ser and ser.is_open:
        ser.write(msg.data.encode('utf-8'))  # 将字符串编码成字节并发送
        rospy.loginfo(f"Sent to STM32: {msg.data}")
    else:
        rospy.logwarn("Serial port not open, cannot send data.")

def main():
    global ser

    # 初始化 ROS 节点
    rospy.init_node('YS_H7Multi_usart_node', anonymous=False)

    # 订阅指令话题
    rospy.Subscriber("/stm32_cmd", String, write_to_stm32)

    # 串口参数设置
    port_name = "/dev/ttySTM32"  # 改成你实际使用的串口名
    baudrate = 115200

    try:
        ser = serial.Serial(port=port_name, baudrate=baudrate, timeout=1)
        rospy.loginfo("Serial port initialized.")
        # 可以在此处发送启动指令 ser.write(b"INIT\r\n")
    except serial.SerialException as e:
        rospy.logerr(f"Unable to open port {port_name}: {e}")
        return

    # 保持节点运行
    rospy.spin()

    # 节点关闭时关闭串口
    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()
