#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sciurus17_gripper_interfaces.srv import SetHandAngle
import sys

class HandAngleClient(Node):
    def __init__(self, namespace):
        super().__init__('hand_angle_client')
        self.namespace = namespace
        service_name = f'{self.namespace}/set_hand_angle'
        self.cli = self.create_client(SetHandAngle, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'サービス "{service_name}" が利用可能になるまで待っています...')
        self.req = SetHandAngle.Request()

    def send_request(self, angle):
        self.req.angle = angle
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    # 引数処理：引数が 'right' のときのみ右を使う
    side = sys.argv[1] if len(sys.argv) > 1 else 'left'
    if side == 'right':
        namespace = '/right'
    else:
        namespace = '/left'

    client = HandAngleClient(namespace)

    angle_value = 0
    client.get_logger().info(f"ハンド角度リクエスト送信: {angle_value} → {namespace}/set_hand_angle")

    future = client.send_request(angle_value)
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        client.get_logger().info(f"サービス応答: {future.result().message}")
    else:
        client.get_logger().error("サービス呼び出しに失敗しました")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
