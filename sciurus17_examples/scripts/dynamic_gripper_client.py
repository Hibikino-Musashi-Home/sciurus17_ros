#!/usr/bin/env python3
"""
dynamic_gripper_client.py  ― 把持力しきい値で停止し、
/gripper/grasp_done (std_srvs/SetBool) を呼び出して把持完了を通知する版
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from xela_server_ros2.msg import ZForceAvg

# ★ 追加: SetBool サービス型
from std_srvs.srv import SetBool
from sciurus17_gripper_interfaces.srv import SetHandAngle

# ---------- 可変パラメータ ----------
ANGLE_START      = 100
TARGET_ANGLE     = 0
ANGLE_STEP       = 3
SLEEP_SEC        = 0.05
FORCE_THRESHOLD  = 0.5
# -----------------------------------


class ForceAwareGripperClient(Node):
    def __init__(self, side='left'):
        super().__init__('force_aware_gripper_client')

        # ------- ハンド角度サービス -------
        ns = '/left' if side == 'left' else '/right'
        angle_srv_name = f'{ns}/set_hand_angle'
        self.cli = self.create_client(SetHandAngle, angle_srv_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'サービス "{angle_srv_name}" を待機中…')
        self.angle_req = SetHandAngle.Request()

        # ------- 把持完了サービス (SetBool) -------
        self.done_cli = self.create_client(SetBool, 'gripper/grasp_done')
        self.done_req = SetBool.Request()          # Request は data(bool)1つ
        self.done_req.data = True                 # ← 常に True を送る

        # ------- そのほか -------
        self.current_angle = ANGLE_START
        self.angle_pub = self.create_publisher(Int32, 'gripper/current_angle', 10)
        self.force_exceeded = False
        self.latest_force_z = 0.0
        self.create_subscription(ZForceAvg, 'z_force_avg', self.force_cb, 10)

    # --- コールバック ---
    def force_cb(self, msg: ZForceAvg):
        self.latest_force_z = msg.z_avg
        if msg.z_avg > FORCE_THRESHOLD and not self.force_exceeded:
            self.force_exceeded = True
            self.get_logger().warn(
                f'z_avg={msg.z_avg:.3f} > {FORCE_THRESHOLD} → 停止')

    # --- 角度送信 ---
    def send_angle(self, angle: int) -> bool:
        self.angle_req.angle = angle
        future = self.cli.call_async(self.angle_req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None or not future.result().success:
            self.get_logger().error('SetHandAngle 呼び出し失敗')
            return False

        msg = Int32()
        msg.data = angle
        self.angle_pub.publish(msg)
        self.current_angle = angle
        return True

    # --- 把持完了サービス呼び出し ---
    def notify_grasp_done(self):
        future = self.done_cli.call_async(self.done_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None or not future.result().success:
            self.get_logger().error('/gripper/grasp_done 呼び出し失敗')
        else:
            self.get_logger().info('/gripper/grasp_done サービス呼び出し成功')

    # --- メイン処理 ---
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

        # 最終角度を publish（任意）
        final_angle_msg = Int32()
        final_angle_msg.data = self.current_angle
        self.angle_pub.publish(final_angle_msg)

        # ★ 把持完了サービスを送信
        self.notify_grasp_done()

        self.get_logger().info(f'完了: 最終角度 = {self.current_angle}°')


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
