#!/usr/bin/env python3
import rospy
import numpy as np # type: ignore
import pinocchio as pin
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped

# 机器人模型加载
model = pin.buildModelFromUrdf(
    "/home/xlh/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
)
data  = model.createData()
# 末端执行器对应的 joint 在 pinocchio 模型里的索引，默认 6
ee_joint_id = int(rospy.get_param("~ee_joint_id", 6))
# 定义模型中的关节名字列表
model_joint_names = model.names[1:]
frame_id    = rospy.get_param("~ee_frame_id", "base_link")




def js_cb(js: JointState):

    '''
    功能: JointState 回调函数,将 ROS 中的关节角度映射到 Pinocchio 模型顺序,做一次正运动学 (FK)
    输入: /joint_states 消息
    输出: 计算并打印末端执行器在 world 下的位姿 (位置 + 四元数)
    '''
    # 将 JointState 的 name/position 对齐到模型顺序
    name2pos = dict(zip(js.name, js.position))
    # 生成和模型顺序一致的关节角数组 q
    # 如果 ROS 消息里缺少某个关节，就用 0 填充
    q = np.array([float(name2pos.get(n, 0.0)) for n in model_joint_names])

    # FK
    pin.forwardKinematics(model, data, q)
    oMi = data.oMi[ee_joint_id]  # 末端相对 world 的位姿

    # 打印：位置 + 四元数
    T = np.eye(4)
    T[:3,:3] = oMi.rotation
    T[:3, 3] = oMi.translation
    qx, qy, qz, qw = quaternion_from_matrix(T)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(oMi.translation[0])
    msg.pose.position.y = float(oMi.translation[1])
    msg.pose.position.z = float(oMi.translation[2])
    msg.pose.orientation.x = float(qx)
    msg.pose.orientation.y = float(qy)
    msg.pose.orientation.z = float(qz)
    msg.pose.orientation.w = float(qw)

    pub_ee.publish(msg)

    # rospy.loginfo_throttle(
    #     0.5,
    #     "EE Pose | frame=%s pos=[%.6f %.6f %.6f] quat=[%.6f %.6f %.6f %.6f]",
    #     msg.header.frame_id,
    #     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
    #     msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
    # )



def main():
    '''
    功能: 节点主入口
    '''
    global pub_ee

    rospy.init_node("fk_node")

    # 可配置订阅的关节话题 & 发布话题名
    joint_topic  = rospy.get_param("~joint_topic", "/joint_states")
    ee_topic     = rospy.get_param("~ee_topic", "/ee_pose")

    pub_ee = rospy.Publisher(ee_topic, PoseStamped, queue_size=10)
    rospy.Subscriber(joint_topic, JointState, js_cb, queue_size=10)

    # rospy.loginfo("FK最简节点就绪：订阅 %s，URDF=%s，ee_joint_id=%d", joint_topic, urdf, ee_joint_id)
    rospy.spin()

if __name__ == "__main__":
    main()
