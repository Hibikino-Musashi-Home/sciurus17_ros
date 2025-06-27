#!/usr/bin/env python3
import math
import sys
from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32
from sciurus17_gripper_interfaces.srv import SetHandAngle
from xela_server_ros2.msg import SensStream

# -------------------- パラメータ -------------------- #
SLIP_T_TH         = 0.015
SLIP_RATIO_TH     = 0.35
CONTACT_N_MIN     = 0.02
WINDOW            = 3
CLOSE_STEP_DEG    = 1
MIN_ANGLE_DEG     = 0
SEND_INTERVAL_SEC = 0.05
SETTLE_SEC        = 0.3
# ---------------------------------------------------- #

class SlipControlGripper(Node):
    def __init__(self, hand_side: str):
        super().__init__('slip_control_gripper_' + hand_side)
        service_ns = f"/{hand_side}/set_hand_angle"

        # --- 状態
        self.enabled = False
        self.settle_deadline: Optional[float] = None
        self.current_angle = 100
        self.last_send_time = self.get_clock().now()
        self.busy = False
        self.pending_angle: Optional[int] = None

        # --- 通信
        self.create_subscription(Bool, 'gripper/grasp_done', self.trigger_cb, 10)
        self.create_subscription(Int32, 'gripper/current_angle', self.angle_cb, 10)
        self.create_subscription(SensStream, 'xServTopic', self.tactile_cb, 10)

        self.cli = self.create_client(SetHandAngle, service_ns)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'サービス {service_ns} を待機中…')
        self.req = SetHandAngle.Request()

        self.force_buf: deque[tuple[float, float]] = deque(maxlen=WINDOW)
        self.create_timer(0.02, self.slip_check)

    def trigger_cb(self, msg: Bool):
        if msg.data and not self.enabled:
            self.enabled = True
            self.force_buf.clear()
            self.settle_deadline = self.get_clock().now().nanoseconds * 1e-9 + SETTLE_SEC
            self.get_logger().info(f'grasp_done 受信 → {SETTLE_SEC}s 待機後に滑り監視開始')

    def angle_cb(self, msg: Int32):
        self.current_angle = int(msg.data)

    def tactile_cb(self, msg: SensStream):
        if not self.enabled:
            return
        t_sum = n_sum = cnt = 0
        for s in msg.sensors:
            for f in s.forces:
                t_sum += math.hypot(f.x, f.y)
                n_sum += max(-f.z, 0.0)
                cnt += 1
        if cnt:
            self.force_buf.append((t_sum / cnt, n_sum / cnt))

    def slip_check(self):
        if not self.enabled or len(self.force_buf) < WINDOW:
            return
        if self.settle_deadline and self.get_clock().now().nanoseconds * 1e-9 < self.settle_deadline:
            return
        self.settle_deadline = None

        t_avg = sum(t for t, _ in self.force_buf) / WINDOW
        n_avg = sum(n for _, n in self.force_buf) / WINDOW
        ratio = t_avg / (n_avg + 1e-6)

        if n_avg < CONTACT_N_MIN:
            return
        if t_avg > SLIP_T_TH and ratio > SLIP_RATIO_TH:
            self.get_logger().warn(f'SLIP! t={t_avg:.3f} N n={n_avg:.3f} μ≈{ratio:.2f}')
            self.close_a_bit()

    def close_a_bit(self):
        if self.current_angle <= MIN_ANGLE_DEG:
            self.get_logger().info('最小角度 reached')
            return
        if self.busy:
            self.pending_angle = max(self.current_angle - CLOSE_STEP_DEG, MIN_ANGLE_DEG)
            return
        now = self.get_clock().now()
        if (now - self.last_send_time).nanoseconds * 1e-9 < SEND_INTERVAL_SEC:
            return
        self.last_send_time = now
        new_angle = max(self.current_angle - CLOSE_STEP_DEG, MIN_ANGLE_DEG)
        self.send_angle_async(new_angle)

    def send_angle_async(self, angle: int):
        angle = int(min(max(angle, 0), 100))
        if angle == self.current_angle:
            return
        self.req.angle = angle
        self.get_logger().info(f'>>> set_hand_angle: {angle}')
        self.busy = True
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        self.busy = False
        if future.result() is None:
            self.get_logger().error('set_hand_angle 応答なし')
            self.pending_angle = None
            return
        if future.result().success:
            self.current_angle = int(self.req.angle)
            self.get_logger().info(f'<<< 成功 | current_angle={self.current_angle}')
            self.force_buf.clear()
        else:
            self.get_logger().error(f'set_hand_angle 失敗: {future.result().message}')
        if self.pending_angle is not None:
            next_angle = self.pending_angle
            self.pending_angle = None
            self.send_angle_async(next_angle)

# ------------------------- main ------------------------- #

def main(args=None):
    rclpy.init(args=args)
    hand_side = "left"
    if len(sys.argv) > 1 and sys.argv[1] in ["left", "right"]:
        hand_side = sys.argv[1]
    node = SlipControlGripper(hand_side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
