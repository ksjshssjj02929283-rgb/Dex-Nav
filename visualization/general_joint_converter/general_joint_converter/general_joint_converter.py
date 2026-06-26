import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GeneralJointConverter(Node):
    def __init__(self):
        super().__init__('general_joint_converter')
        
        # 声明参数
        self.declare_parameter('use_simulation', False)
        use_simulation = self.get_parameter('use_simulation').get_parameter_value().bool_value
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # 根据仿真模式设置话题名
        if use_simulation:
            input_topic = '/feedback/robot_server_state_sim'
            self.get_logger().info('启动仿真模式')
        else:
            input_topic = '/feedback/robot_server_state'
            self.get_logger().info('启动真实模式')

        # 订阅 String 话题（根据模式使用不同话题）
        self.sub = self.create_subscription(
            String,
            input_topic, 
            self.callback,
            qos_profile
        )

        # 发布 JointState（统一发布到 /joint_states）
        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            1
        )
        
        self.pub_time = 0
        self.pub_gap = 10

        # 调试信息
        self.get_logger().info(f'订阅话题: {input_topic}')
        self.get_logger().info(f'发布话题: /joint_states')
        self.get_logger().info('GeneralJointConverter 节点初始化完成')

        # 按 URDF 里的顺序设置关节名
        self.joint_names = [
            "ANKLE",
            "KNEE", 
            "BUTTOCK",
            "WAIST",
            "NECK1",
            "NECK2",
            "LEFT_J1",
            "LEFT_J2",
            "LEFT_J3",
            "LEFT_J4",
            "LEFT_J5",
            "LEFT_J6",
            "LEFT_J7",
            "RIGHT_J1",
            "RIGHT_J2",
            "RIGHT_J3",
            "RIGHT_J4",
            "RIGHT_J5",
            "RIGHT_J6",
            "RIGHT_J7"
        ]

        self.joint_msg = JointState()
        self.joint_msg.name = self.joint_names

    def callback(self, msg):
        if (self.pub_gap == self.pub_time):
            self.pub_time = 0
            try:
                # 解析 JSON
                data = json.loads(msg.data)
                positions = data["joint_position"]
                # 确认数据长度
                if len(positions) != len(self.joint_names):
                    self.get_logger().warn(f"关节数量不匹配: {len(positions)} vs {len(self.joint_names)}")
                    return
                # 构造 JointState 消息
                self.joint_msg.header.stamp = self.get_clock().now().to_msg()
                # 确保 position 是 float 列表
                self.joint_msg.position = [float(x) for x in positions]
                self.pub.publish(self.joint_msg)

            except json.JSONDecodeError as e:
                self.get_logger().error(f"JSON 解析失败: {e}")
            except Exception as e:
                self.get_logger().error(f"处理消息时出错: {e}")
        else:
            self.pub_time = self.pub_time + 1

def main(args=None):
    rclpy.init(args=args)
    node = GeneralJointConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

