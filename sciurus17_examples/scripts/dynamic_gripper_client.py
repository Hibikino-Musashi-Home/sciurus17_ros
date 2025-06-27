#!/usr/bin/env python3
"""
dynamic_gripper_client.py

- gripper_server.cpp の /left/set_hand_angle, /right/set_hand_angle に対応
- 実行引数で対象側（left/right）を切替可能（デフォルトは left）
"""

import sys
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

from xela_server_ros2.msg import ZForceAvg
from sciurus17_gripper_interfaces.srv import SetHandAngle

# ---------- 可変パラメータ ----------
ANGLE_START      = 100
TARGET_ANGLE     = 0
ANGLE_STEP       = 3
SLEEP_SEC        = 0.05
FORCE_THRESHOLD  = 0.09
# -----------------------------------


class ForceAwareGripperClient(Node):
    def __init__(self, side='left'):
        super().__init__('force_aware_gripper_client')

        # 利き腕の設定
        ns = '/left' if side == 'left' else '/right'
        service_name = f'{ns}/set_hand_angle'

        self.get_logger().info(f'Service target: {service_name}')
        self.current_angle = ANGLE_START

        # サービスクライアント
        self.cli = self.create_client(SetHandAngle, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'サービス "{service_name}" を待機中…')
        self.req = SetHandAngle.Request()

        # 角度ステータス Publisher（任意）
        self.angle_pub = self.create_publisher(Int32, 'gripper/current_angle', 10)

        # 力センサ Subscriber
        self.force_exceeded = False
        self.latest_force_z = 0.0
        self.create_subscription(ZForceAvg, 'z_force_avg', self.force_cb, 10)

        self.done_pub = self.create_publisher(Bool, 'gripper/grasp_done', 1)

    def force_cb(self, msg: ZForceAvg):
        self.latest_force_z = msg.z_avg
        if msg.z_avg > FORCE_THRESHOLD and not self.force_exceeded:
            self.force_exceeded = True
            self.get_logger().warn(
                f'z_avg= {msg.z_avg:.3f} > {FORCE_THRESHOLD} → 停止')

    def send_angle(self, angle: int) -> bool:
        self.req.angle = angle
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None or not future.result().success:
            self.get_logger().error('SetHandAngle 呼び出し失敗')
            return False

        msg = Int32()
        msg.data = angle
        self.angle_pub.publish(msg)
        self.current_angle = angle
        return True

    def run(self):
        self.get_logger().info(f'開始: {ANGLE_START}° → {TARGET_ANGLE}°')

        for angle in range(ANGLE_START, TARGET_ANGLE - 1, -ANGLE_STEP):
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.force_exceeded:
                self.get_logger().info('停止条件達成 → 終了')
                break

            self.get_logger().info(f'→ 角度 = {angle}, z = {self.latest_force_z:.3f}')
            if not self.send_angle(angle):
                break
            time.sleep(SLEEP_SEC)

        final_angle_msg = Int32()
        final_angle_msg.data = self.current_angle
        self.angle_pub.publish(final_angle_msg)

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

        self.get_logger().info(f'完了: 最終角度 = {self.current_angle}°, grasp_done 発行')


def main(args=None):
    rclpy.init(args=args)

    # コマンドライン引数から side を取得（デフォルト: left）
    side = 'left'
    if len(sys.argv) > 1 and sys.argv[1] in ['left', 'right']:
        side = sys.argv[1]

    node = ForceAwareGripperClient(side)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
