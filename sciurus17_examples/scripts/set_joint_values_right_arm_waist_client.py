#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sciurus17_interfaces.action import SetJointValues
import sys

class SetJointValuesRightArmWaist(Node):
    def __init__(self):
        super().__init__('set_joint_values_client')
        self._client = ActionClient(self, SetJointValues, 'set_joint_values_right_arm_waist')

    def send_goal(self, joint_values):
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        goal_msg = SetJointValues.Goal()
        goal_msg.joint_values = joint_values

        self.get_logger().info(f'Sending joint values: {joint_values}')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted. Waiting for result...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        self.get_logger().info(f'Result: success={result.success}, message="{result.message}"')

        return result.success


def main():
    rclpy.init()
    client = SetJointValuesRightArmWaist()

    # 任意の関節角度（ラジアン）
    joint_values = [1.0, -0.291456, -1.58153, 0.101243, 2.71821, 0.0291456, -1.57386, -0.0122718]

    success = client.send_goal(joint_values)
    client.destroy_node()
    rclpy.shutdown()

    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
