#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
from end_effector_interfaces.msg import (
    EEJointControl,
    EEJointControlMode,
    EEFeedback
)

class DexHandJointConverter(Node):
    def __init__(self):
        super().__init__('dexhand_joint_converter')

        # 声明参数
        self.declare_parameter('use_simulation', False)
        use_simulation = self.get_parameter('use_simulation').get_parameter_value().bool_value
        joint_state_topic = '/joint_states'

        # 根据仿真模式设置话题名
        if use_simulation:
            left_topic = '/feedback_sim/ee/left'
            right_topic = '/feedback_sim/ee/right'
            self.get_logger().info('启动仿真模式')
        else:
            left_topic = '/feedback/ee/left'
            right_topic = '/feedback/ee/right'
            self.get_logger().info('启动真实模式')

        # 统一发布到 /joint_states，让robot_state_publisher自动订阅
        joint_state_topic = '/joint_states'

        # 左右手主动关节名（与URDF一致）
        self.left_joint_names = [
            "LEFT_L_THUMBMCP2",   # 拇指弯曲
            "LEFT_L_THUMBMCP1",   # 拇指摆动
            "LEFT_L_INDEXMCP",    # 食指弯曲
            "LEFT_L_MIDDLEMCP",   # 中指弯曲
            "LEFT_L_RINGMCP",     # 无名指弯曲
            "LEFT_L_LITTLEMCP"    # 小指弯曲
        ]
        self.right_joint_names = [
            "RIGHT_R_THUMBMCP2",  # 拇指弯曲
            "RIGHT_R_THUMBMCP1",  # 拇指摆动
            "RIGHT_R_INDEXMCP",   # 食指弯曲
            "RIGHT_R_MIDDLEMCP",  # 中指弯曲
            "RIGHT_R_RINGMCP",    # 无名指弯曲
            "RIGHT_R_LITTLEMCP"   # 小指弯曲
        ]

        # 每个关节的范围 (rad)
        self.left_joint_ranges = [
            (0.0, math.radians(90.0)),   # THUMBMCP1
            (0.0, math.radians(50.0)),   # THUMBMCP2
            (0.0, math.radians(75.0)),   # INDEXMCP
            (0.0, math.radians(75.0)),   # MIDDLEMCP
            (0.0, math.radians(75.0)),   # RINGMCP
            (0.0, math.radians(75.0))    # LITTLEMCP
        ]
        self.right_joint_ranges = [
            (0.0, math.radians(90.0)),   # THUMBMCP1
            (0.0, math.radians(50.0)),   # THUMBMCP2
            (0.0, math.radians(75.0)),   # INDEXMCP
            (0.0, math.radians(75.0)),   # MIDDLEMCP
            (0.0, math.radians(75.0)),   # RINGMCP
            (0.0, math.radians(75.0))    # LITTLEMCP
        ]


        # 订阅左右手百分比输入（根据模式使用不同话题）
        self.sub_left = self.create_subscription(
            EEFeedback,
            left_topic,
            self.callback_left,
            10
        )
        self.sub_right = self.create_subscription(
            EEFeedback,
            right_topic,
            self.callback_right,
            10
        )

        # 发布 JointState（根据模式使用不同话题）
        self.pub = self.create_publisher(JointState, joint_state_topic, 10)

        # 调试信息
        self.get_logger().info(f'订阅话题: {left_topic}, {right_topic}')
        self.get_logger().info(f'发布话题: {joint_state_topic}')
        self.get_logger().info('DexHandJointConverter 节点初始化完成')

    def percent_to_rad(self, percent_list, joint_ranges):
        """
        将百分比列表转换为弧度列表
        Args:
            percent_list: 百分比值列表 (0-100)
            joint_ranges: 关节范围列表 [(min_rad, max_rad), ...]

        Returns:
            弧度值列表
        """
        rad_list = []
        for i, p in enumerate(percent_list):
            min_angle, max_angle = joint_ranges[i]
            # joint_ranges已经是弧度，直接按百分比线性插值
            rad = min_angle + (max_angle - min_angle) * (p / 100.0)
            rad_list.append(rad)
        return rad_list

    def callback_left(self, msg: EEFeedback):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.left_joint_names
        positions = self.percent_to_rad(
            [j.position for j in msg.joint_states], self.left_joint_ranges)
        js.position = positions
        self.pub.publish(js)

    def callback_right(self, msg: EEFeedback):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.right_joint_names
        positions = self.percent_to_rad(
            [j.position for j in msg.joint_states], self.right_joint_ranges)
        js.position = positions
        self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = DexHandJointConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()