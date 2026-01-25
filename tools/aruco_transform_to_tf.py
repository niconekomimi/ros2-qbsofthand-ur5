#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TransformStampedToTf(Node):
    def __init__(self, input_topic: str):
        super().__init__("transform_stamped_to_tf")
        self._broadcaster = TransformBroadcaster(self)
        self._input_topic = input_topic
        self._count = 0
        self._first_received = False
        
        self.get_logger().info(f"启动 TF 转发节点")
        self.get_logger().info(f"  订阅话题: {input_topic}")
        self.get_logger().info(f"  发布到: /tf")
        self.get_logger().info("等待消息...")
        
        self._sub = self.create_subscription(
            TransformStamped,
            input_topic,
            self._on_transform,
            10,
        )

    def _on_transform(self, msg: TransformStamped) -> None:
        if not msg.header.frame_id or not msg.child_frame_id:
            self.get_logger().warn("Received TransformStamped with empty frame_id/child_frame_id")
            return

        if not self._first_received:
            self.get_logger().info(
                f"✓ 开始转发: {msg.header.frame_id} -> {msg.child_frame_id}"
            )
            self._first_received = True

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id
        out.transform = msg.transform

        self._broadcaster.sendTransform(out)
        
        self._count += 1
        if self._count % 30 == 0:  # 每30条打印一次
            self.get_logger().info(f"已转发 {self._count} 条变换")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Relay geometry_msgs/TransformStamped (e.g. /aruco_single/transform) into TF (/tf)."
    )
    parser.add_argument(
        "--input-topic",
        default="/aruco_single/transform",
        help="TransformStamped topic to subscribe to (default: /aruco_single/transform)",
    )

    args = parser.parse_args()

    rclpy.init()
    node = TransformStampedToTf(args.input_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # In some environments shutdown can be triggered twice (e.g. when SIGINT
            # affects a whole process group). Ignore and exit cleanly.
            pass


if __name__ == "__main__":
    main()
