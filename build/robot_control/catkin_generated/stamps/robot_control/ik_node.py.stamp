#!/usr/bin/env python3
import rospy
import numpy as np  # type: ignore # 数值计算库
import pinocchio as pin  # 机器人运动学库
from numpy.linalg import norm, solve  # type: ignore # 矩阵范数与线性方程求解
from geometry_msgs.msg import PoseStamped  # ROS 消息: 位姿(位置+姿态)
from sensor_msgs.msg import JointState     # ROS 消息: 关节状态
from tf.transformations import quaternion_matrix  # 四元数 → 齐次矩阵


def wrap_to_pi(angle):
    """角度归一化到 [-pi, pi]，避免角度跳变"""
    return ((angle + np.pi) % (2 * np.pi)) - np.pi


class IKSolver:
    """
    逆运动学解算器（数值法：阻尼最小二乘 DLS）。
    - 从 ROS 参数服务器读取参数
    - 使用 Pinocchio 完成 FK、Jacobian 计算
    - 提供 solve() 函数迭代求解 IK
    - ROS 接口：订阅目标末端位姿 + 当前关节状态，发布 IK 解算的关节角
    """

    def __init__(self):
        '''
        初始化 IK 解算器:
          1. 读取参数 (URDF 路径、关节 ID、收敛阈值等)
          2. 构建机器人模型和缓存数据
          3. 设置初始关节角（从 /joint_states 更新）
          4. ROS 通信接口：订阅目标位姿、订阅当前关节状态、发布目标关节角
        '''
        # —— 参数 —— #
        self.urdf_path = rospy.get_param("~urdf_path",  # URDF 路径
            "/home/xlh/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
        )   
        self.joint_id  = rospy.get_param("~joint_id", 6)    # 末端执行器所在关节 ID
        self.eps       = rospy.get_param("~eps", 1e-4)      # 收敛阈值
        self.it_max    = rospy.get_param("~it_max", 1000)   # 最大迭代次数
        self.dt        = rospy.get_param("~dt", 1e-1)       # 积分步长
        self.damp      = rospy.get_param("~damp", 1e-12)    # 阻尼系数

        # —— 模型 —— #
        self.model = pin.buildModelFromUrdf(self.urdf_path)  # 从 URDF 构建模型
        self.data  = self.model.createData()                 # 分配计算缓存
        self.q_current = np.zeros(self.model.nq)             # 当前关节角（从 /joint_states 更新）

        # —— 发布器 —— #
        # 注意: 这里输出的是 IK 目标解，不是真实状态
        self.pub = rospy.Publisher("/ik_joint_targets", JointState, queue_size=100)

        # —— 4. 订阅器 —— #
        # 订阅目标末端位姿
        rospy.Subscriber("/Interpolation_Point", PoseStamped, self.pose_cb)
        # 订阅真实关节角状态（用于 IK 初值）
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)


    def joint_state_cb(self, msg: JointState):
        """
        回调函数: 保存最新的关节角 (q_current)
        - 输入: JointState
        - 输出: 更新 self.q_current
        """
        name2pos = dict(zip(msg.name, msg.position))
        self.q_current = np.array([
            float(name2pos.get(n, 0.0)) for n in self.model.names[1:]
        ])


    def solve(self, oMdes: pin.SE3, q_init=None):
        """
        迭代求解逆运动学
        输入:
          oMdes  - 目标末端位姿 (Pinocchio SE3)
          q_init - 初始关节角 (可选)，默认使用 self.q_current
        输出:
          (success, [q]) - 是否收敛 + 解
        """
        # 初值: 优先用传入的 q_init，否则用当前关节角
        q = q_init.copy() if q_init is not None else self.q_current.copy()
        # rospy.loginfo(f"初始角度: {np.array(q)}")

        i = 0
        while True:
            # 正运动学，计算当前末端位姿
            pin.forwardKinematics(self.model, self.data, q)

            # 计算末端误差 (目标 - 当前)
            iMd = self.data.oMi[self.joint_id].actInv(oMdes)
            err = pin.log(iMd).vector  # 6x1 twist (旋转+平移)

            # 收敛判定
            if norm(err) < self.eps:
                q = np.array([wrap_to_pi(a) for a in q])  # 角度归一化
                self.q_current = q.copy()                 # 更新当前解
                return True, [q]

            # 超过最大迭代次数
            if i >= self.it_max:
                return False, []

            # 计算雅可比
            J = pin.computeJointJacobian(self.model, self.data, q, self.joint_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)

            # 阻尼最小二乘迭代更新
            v = -J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * self.dt)

            i += 1


    def pose_cb(self, msg: PoseStamped):
        """
        回调函数: 接收目标末端位姿，调用 IK 解算并发布目标关节角
        输入: PoseStamped (目标末端位姿)
        输出: JointState (目标关节角)
        """
        # PoseStamped → Pinocchio SE3
        p = msg.pose.position
        q = msg.pose.orientation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [p.x, p.y, p.z]
        oMdes = pin.SE3(T[0:3, 0:3], T[0:3, 3])

        # IK 解算
        success, sols = self.solve(oMdes, q_init=None)
        if not success or not sols:
            rospy.logwarn("[ERROR] IK 解算失败，未收敛。")
            return

        best_q = sols[0]

        # 发布 JointState (目标关节角)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.model.names[1:]
        js.position = best_q.tolist()
        self.pub.publish(js)

        # rospy.loginfo(f"IK 解算结果(关节角): {np.array(best_q).round(3)}")


def main():
    rospy.init_node("ik_node")  # 节点名
    IKSolver()                  # 创建实例，自动注册订阅器和发布器
    rospy.spin()                 # 循环等待回调


if __name__ == "__main__":
    main()
