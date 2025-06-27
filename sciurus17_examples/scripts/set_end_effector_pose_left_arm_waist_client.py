#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sciurus17_interfaces.action import SetPose
from geometry_msgs.msg import Pose
import sys
import time

class SetEndEffectorPoseLeftArmWaist(Node):
    def __init__(self):
        super().__init__('set_pose_client')
        self._client = ActionClient(self, SetPose, 'set_end_effector_pose_left_arm_waist')

    def send_pose_goal(self, pose: Pose):
        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        goal_msg = SetPose.Goal()
        goal_msg.target_pose = pose

        self.get_logger().info('Sending goal...')
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
    client = SetEndEffectorPoseLeftArmWaist()

    # 任意の目標姿勢
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = 0.0
    pose.position.z = 0.3
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    success = client.send_pose_goal(pose)
    client.destroy_node()
    rclpy.shutdown()

    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
