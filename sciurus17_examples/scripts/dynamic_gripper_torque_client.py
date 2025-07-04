#!/usr/bin/env python3
"""
dynamic_gripper_torque_client.py

- /left|right/set_hand_torque サービスを呼び出して
  一定ステップでトルクを増加させながら把持。
- 力センサ z_avg > THRESHOLD で停止し grasp_done を publish。
"""

import sys, time, rclpy
from rclpy.node   import Node
from std_msgs.msg import Float64, Bool
from xela_server_ros2.msg import ZForceAvg
from sciurus17_gripper_interfaces.srv import SetHandTorque

# ---------- 調整パラメータ ----------
TORQUE_START    = 0.0     # N·m
TARGET_TORQUE   = 0.4    # N·m （閉じる方向）
TORQUE_STEP     = 0.03    # N·m
SLEEP_SEC       = 0.05
FORCE_THRESHOLD = 0.4     # z_avg [N]
# -----------------------------------

class ForceAwareTorqueClient(Node):
    def __init__(self, side='left'):
        super().__init__('force_aware_torque_client')

        ns = '/left' if side == 'left' else '/right'
        srv_name = f'{ns}/set_hand_torque'
        self.get_logger().info(f'Service target: {srv_name}')

        # サービスクライアント
        self.cli = self.create_client(SetHandTorque, srv_name)
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info(f'waiting for "{srv_name}" …')
        self.req = SetHandTorque.Request()

        # 力センサ
        self.force_exceeded = False
        self.latest_force_z = 0.0
        self.create_subscription(ZForceAvg, 'z_force_avg', self.force_cb, 10)

        # 完了通知
        self.done_pub = self.create_publisher(Bool, 'gripper/grasp_done', 1)

    # ----- callback -----
    def force_cb(self, msg: ZForceAvg):
        self.latest_force_z = msg.z_avg
        if msg.z_avg > FORCE_THRESHOLD and not self.force_exceeded:
            self.force_exceeded = True
            self.get_logger().warn(f'z_avg={msg.z_avg:.3f} > {FORCE_THRESHOLD} → stop')

    # ----- helper -----
    def send_torque(self, torque: float) -> bool:
        self.req.torque = torque
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None or not future.result().success:
            self.get_logger().error('SetHandTorque failed')
            return False
        return True

    # ----- main loop -----
    def run(self):
        self.get_logger().info(f'Start: {TORQUE_START} → {TARGET_TORQUE} N·m')

        t = TORQUE_START
        while t <= TARGET_TORQUE:
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.force_exceeded:
                self.get_logger().info('force limit reached → exit')
                break

            self.get_logger().info(f'→ torque={t:.2f}, z={self.latest_force_z:.3f}')
            if not self.send_torque(t):
                break
            t += TORQUE_STEP
            time.sleep(SLEEP_SEC)

        self.done_pub.publish(Bool(data=True))
        self.get_logger().info('grasp_done published, finished.')

def main(args=None):
    rclpy.init(args=args)
    side = 'left'
    if len(sys.argv) > 1 and sys.argv[1] in ['left', 'right']:
        side = sys.argv[1]
    node = ForceAwareTorqueClient(side)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
