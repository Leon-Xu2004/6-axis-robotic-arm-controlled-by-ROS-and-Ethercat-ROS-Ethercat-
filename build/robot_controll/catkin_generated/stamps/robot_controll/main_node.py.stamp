#!/usr/bin/env python3
import rospy
import threading
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class MainController:
    """
    主控制器节点：
      - 订阅：
          ~/start_cmd_topic   (默认 /motion_kinematics/ee_pose)         —— 当前末端位姿（作为起点来源）
          ~/target_cmd_topic  (默认 /target_pose_cmd)                   —— 目标末端位姿命令
      - 发布：
          ~/start_pose_topic  (默认 /start_pose)                        —— 
          ~/target_pose_topic (默认 /target_pose)                       —— 

    逻辑：接到 target 后“开门”，等待下一帧 ee 到来再一次性同步发布 start/target，随后关门。
    注：不额外新增成员变量，复用 Switch_Enable 作为“缓存槽位”。
    """

    def __init__(self):
        # —— 可配置话题名 —— #
        self.start_cmd_topic    = rospy.get_param("~start_cmd_topic", "/motion_kinematics/ee_pose")
        self.target_cmd_topic   = rospy.get_param("~target_cmd_topic", "/target_pose_cmd")
        self.start_pose_topic   = rospy.get_param("~start_pose_topic", "/start_pose")
        self.target_pose_topic  = rospy.get_param("~target_pose_topic", "/target_pose")
        # self.enable_topic      = rospy.get_param("~enable_topic", "/motion_enable")

        # —— 发布器 —— #
        self.pub_start  = rospy.Publisher(self.start_pose_topic,  PoseStamped, queue_size=10)
        self.pub_target = rospy.Publisher(self.target_pose_topic, PoseStamped,  queue_size=10)

        # —— 状态变量 —— #
        self._lock = threading.RLock()
        # 复用 Switch_Enable 作为“缓存”：
        #   None 表示关门/无缓存；
        #   PoseStamped 表示已缓存目标、等待下一帧 ee 触发。
        self.Switch_Enable = None

        # —— 订阅器 —— #
        rospy.Subscriber(self.start_cmd_topic,   PoseStamped, self.ee_cb,     queue_size=20)
        rospy.Subscriber(self.target_cmd_topic,  PoseStamped, self.target_cb, queue_size=10)

    # 回调函数
    def target_cb(self, msg: PoseStamped):
        """接收目标：只缓存，不发布；开门等待下一帧 ee。"""
        with self._lock:
            self.Switch_Enable = msg  # 缓存目标（用作“开门”信号与数据承载）

    def ee_cb(self, msg: PoseStamped):
        """
        高频回调：当且仅当缓存里有目标（门已开）时，用当前这一帧 ee 作为 start，
        与缓存的 target 同时戳发布；随后清空缓存（关门）。
        """
        with self._lock:
            if self.Switch_Enable is None:
                return

            now = rospy.Time.now()

            # —— 构造并发布 start（来源：本帧 ee） —— #
            start_msg = PoseStamped()
            start_msg.header.stamp    = now
            start_msg.header.frame_id = msg.header.frame_id
            start_msg.pose            = msg.pose
            self.pub_start.publish(start_msg)

            # —— 构造并发布 target（来源：缓存的目标） —— #
            tgt = self.Switch_Enable
            target_msg = PoseStamped()
            target_msg.header.stamp    = now  # 与 start 同一时间戳
            target_msg.header.frame_id = tgt.header.frame_id
            target_msg.pose            = tgt.pose
            self.pub_target.publish(target_msg)

            # —— 关门（清空缓存） —— #
            self.Switch_Enable = None


def main():
    rospy.init_node("main_controller")
    MainController()
    rospy.spin()

if __name__ == "__main__":
    main()