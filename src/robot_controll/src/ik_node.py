#!/usr/bin/env python3
import rospy
import numpy as np  # type: ignore
import pinocchio as pin
from numpy.linalg import norm, solve  # type: ignore
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_matrix


def wrap_to_pi(angle):
    """角度归一化到 [-pi, pi]，避免角度跳变"""
    return ((angle + np.pi) % (2 * np.pi)) - np.pi


class IKSolver:
    """
    数值 IK（阻尼最小二乘）。参数全部在 __init__ 中从 ROS 参数服务器读取。
    """

    def __init__(self):
        '''
        初始化 IK 解算器类：
            - 从 ROS 参数服务器读取配置参数（URDF 路径、关节 ID、迭代阈值等）
            - 构建 Pinocchio 模型与数据结构
            - 初始化当前关节角 (q_current)，用于 IK 初值
            - 配置 ROS 通信接口（订阅目标位姿、订阅关节状态、发布关节角解）
        '''
        # —— 从 ROS 参数服务器读取 ——
        self.urdf_path = rospy.get_param("~urdf_path",
            "/home/leon-xu/eRob3_ws/src/robot_description/urdf/erobot3.urdf"
        )   
        self.joint_id  = rospy.get_param("~joint_id", 6)    # 末端执行器所在的关节 ID 
        self.eps       = rospy.get_param("~eps", 1e-4)      # 收敛阈值
        self.it_max    = rospy.get_param("~it_max", 1000)   # 最大迭代次数
        self.dt        = rospy.get_param("~dt", 1e-1)       # 积分步长
        self.damp      = rospy.get_param("~damp", 1e-12)    # 阻尼系数

        # —— 模型与初始关节角 ——
        self.model = pin.buildModelFromUrdf(self.urdf_path)  # 从 URDF 构建模型
        self.data  = self.model.createData()                 # 分配计算缓存
        self.q_current = np.zeros(self.model.nq)             # 当前关节角（从 /joint_states 更新）

        # —— 发布器 —— #
        self.pub = rospy.Publisher("/motion_planning/q_goal", JointState, queue_size=10)

        # —— 订阅器 —— #
        rospy.Subscriber("/target_pose", PoseStamped, self.pose_cb)  # 订阅目标位姿
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)  # 订阅当前关节状态


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
        rospy.loginfo(f"初始角度: {np.array(q)}")

        i = 0
        while True:
            # 1. 正运动学，计算当前末端位姿
            pin.forwardKinematics(self.model, self.data, q)

            # 2. 计算末端误差 (目标 - 当前)
            iMd = self.data.oMi[self.joint_id].actInv(oMdes)
            err = pin.log(iMd).vector  # 6x1 twist (旋转+平移)

            # 3. 收敛判定
            if norm(err) < self.eps:
                q = np.array([wrap_to_pi(a) for a in q])  # 角度归一化
                self.q_current = q.copy()                 # 更新当前解
                return True, [q]

            # 4. 超过最大迭代次数
            if i >= self.it_max:
                return False, []

            # 5. 计算雅可比
            J = pin.computeJointJacobian(self.model, self.data, q, self.joint_id)
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)

            # 6. 阻尼最小二乘迭代更新
            v = -J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pin.integrate(self.model, q, v * self.dt)

            i += 1


    def pose_cb(self, msg: PoseStamped):
        """
        回调函数: 接收目标末端位姿，调用 IK 解算并发布目标关节角
        输入: PoseStamped (目标末端位姿)
        输出: JointState (目标关节角)
        """
        # 1. PoseStamped → Pinocchio SE3
        p = msg.pose.position
        q = msg.pose.orientation
        T = quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = [p.x, p.y, p.z]
        oMdes = pin.SE3(T[0:3, 0:3], T[0:3, 3])

        # 2. IK 解算
        success, sols = self.solve(oMdes, q_init=None)
        if not success or not sols:
            rospy.logwarn("[ERROR] IK 解算失败，未收敛。")
            return

        best_q = sols[0]

        # 3. 发布 JointState (目标关节角)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.model.names[1:]
        js.position = best_q.tolist()
        self.pub.publish(js)

        rospy.loginfo(f"IK 解算结果(关节角): {np.array(best_q).round(3)}")


def main():
    rospy.init_node("ik_node")  # 节点名
    IKSolver()                  # 创建实例，自动注册订阅器和发布器
    rospy.spin()                 # 循环等待回调


if __name__ == "__main__":
    main()