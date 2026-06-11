#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
try:
    from chassis_interfaces.action import AutoCharging
except ImportError:
    from action_interfaces.msg import AutoCharging
from geometry_msgs.msg import PoseStamped, Twist
import random
import time
import sys


class AutoChargingTest(Node):

    def __init__(self, test_count):
        super().__init__('auto_charging_test')

        self.test_count = test_count
        self.current_test = 0
        self.success_count = 0
        self.fail_count = 0

        self.declare_parameter('point_a1',
                               [2.0, -4.0, 0.0, 0.0, 0.0, -0.707, 0.707])
        self.declare_parameter('point_a2',
                               [2.0, -4.0, 0.0, 0.0, 0.0, 0.707, 0.707])
        self.declare_parameter('point_a3',
                               [2.0, -4.0, 0.0, 0.0, 0.0, -0.707, 0.707])

        self.point_a1 = self.get_parameter('point_a1').value
        self.point_a2 = self.get_parameter('point_a2').value
        self.point_a3 = self.get_parameter('point_a3').value

        self.points_a = [self.point_a1, self.point_a2, self.point_a3]

        self.get_logger().info(f'Test count: {self.test_count}')
        self.get_logger().info(f'Point A1: {self.point_a1}')
        self.get_logger().info(f'Point A2: {self.point_a2}')
        self.get_logger().info(f'Point A3: {self.point_a3}')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.charging_client = ActionClient(self, AutoCharging,
                                            '/auto_charging')

        self.cmd_vel_pub = self.create_publisher(Twist, 'control/cmd_vel', 10)

        self.get_logger().info('Waiting for navigation action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Navigation action server connected!')

        self.get_logger().info('Waiting for charging action server...')
        self.charging_client.wait_for_server()
        self.get_logger().info('Charging action server connected!')

        self.start_test()

    def create_pose(self, point):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.x = point[3]
        pose.pose.orientation.y = point[4]
        pose.pose.orientation.z = point[5]
        pose.pose.orientation.w = point[6]
        return pose

    def start_test(self):
        for timer_attr in ['wait_timer', 'move_timer']:
            if hasattr(self, timer_attr):
                timer = getattr(self, timer_attr)
                if timer is not None:
                    try:
                        timer.cancel()
                    except:
                        pass
                    setattr(self, timer_attr, None)

        self.current_test += 1
        self.get_logger().info(
            f'========== Test {self.current_test}/{self.test_count} started =========='
        )

        target_point = random.choice(self.points_a)
        if target_point:
            self.get_logger().info(
                f'Navigating to random point: ({target_point[0]}, {target_point[1]})'
            )
        else:
            self.get_logger().error('No valid navigation point!')
            self.safe_shutdown()
            return

        self.navigate_to_pose(target_point)

    def navigate_to_pose(self, point):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(point)

        self.get_logger().info('Sending navigation goal...')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(f'Navigation feedback: {feedback}')

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.get_logger().info(
                'Navigation goal rejected, continuing to next test...')
            self.next_test()
            return

        self.get_logger().info(
            'Navigation goal accepted! Waiting for navigation to complete...')
        self._nav_goal_handle = goal_handle
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation result: {result}')

            if result and result.result:
                self.get_logger().info(
                    'Navigation succeeded! Starting charging action...')
                self.start_charging()
            else:
                self.get_logger().warn(
                    'Navigation failed! Continuing to next test...')
                self.next_test()
        except Exception as e:
            self.get_logger().error(
                f'Navigation error: {str(e)}. Continuing to next test...')
            self.next_test()

    def start_charging(self):
        self.get_logger().info('Starting auto charging...')

        goal_msg = AutoCharging.Goal()
        goal_msg.force = True

        send_goal_future = self.charging_client.send_goal_async(
            goal_msg, feedback_callback=self.charging_feedback_callback)
        send_goal_future.add_done_callback(self.charging_goal_response_callback)

    def charging_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Charging feedback: current_state={feedback.current_state}, retry_count={feedback.retry_count}'
        )

    def charging_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Charging goal rejected!')
            self.fail_count += 1
            self.next_test()
            return

        self.get_logger().info('Charging goal accepted!')
        self._charging_result_future = goal_handle.get_result_async()
        self._charging_result_future.add_done_callback(
            self.charging_result_callback)

    def charging_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f'Charging result: success={result.success}, message={result.message}'
            )

            if result.success:
                self.get_logger().info('Charging action succeeded!')
                self.success_count += 1
                self.wait_and_move_forward()
            else:
                self.get_logger().error(
                    f'Charging action failed: {result.message}')
                self.fail_count += 1
                self.next_test()
        except Exception as e:
            self.get_logger().error(f'Charging action error: {str(e)}')
            self.fail_count += 1
            self.next_test()

    def wait_and_move_forward(self):
        self.get_logger().info('Waiting 10 seconds before moving forward...')

        self.wait_timer = self.create_timer(10.0, self.move_forward)

    def move_forward(self):
        self.get_logger().info(
            'Moving forward with velocity 0.3 m/s for 2 seconds...')

        twist = Twist()
        twist.linear.x = 0.3
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        self.move_start_time = time.time()
        self.move_timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        elapsed = time.time() - self.move_start_time

        if elapsed >= 2.0:
            self.get_logger().info('Stopping robot...')
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.move_timer.cancel()
            self.get_logger().info('Robot stopped.')
            self.next_test()
        else:
            twist = Twist()
            twist.linear.x = 0.15
            self.cmd_vel_pub.publish(twist)

    def next_test(self):
        self.get_logger().info(
            f'========== Charging Action Summary: Success={self.success_count}, Failed={self.fail_count} =========='
        )

        if self.current_test < self.test_count:
            time.sleep(1)
            self.start_test()
        else:
            self.get_logger().info('========== All tests completed! ==========')
            self.get_logger().info(f'Total tests: {self.test_count}')
            self.get_logger().info(
                f'Charging action success: {self.success_count}, Failed: {self.fail_count}'
            )
            if self.success_count + self.fail_count > 0:
                success_rate = (self.success_count /
                                (self.success_count + self.fail_count)) * 100
                self.get_logger().info(
                    f'Charging action success rate: {success_rate:.2f}%')
            self.safe_shutdown()

    def safe_shutdown(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(description='Auto Charging Test Script')
    parser.add_argument('-c',
                        '--count',
                        type=int,
                        default=5,
                        help='Number of tests to run')
    parsed_args = parser.parse_args()

    test_count = parsed_args.count

    rclpy.init(args=args)

    try:
        test_node = AutoChargingTest(test_count)
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print('Node shutdown by keyboard interrupt')
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
