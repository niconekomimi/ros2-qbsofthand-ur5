#!/usr/bin/env python3
import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


UR_JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class OneShotJointState(Node):
    def __init__(self):
        super().__init__("ur_one_shot_joint_state")
        self._msg: Optional[JointState] = None
        self._sub = self.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        if self._msg is None and msg.name and msg.position:
            self._msg = msg

    def wait(self, timeout_s: float) -> Optional[JointState]:
        end = time.time() + timeout_s
        while rclpy.ok() and time.time() < end and self._msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._msg


def _reorder_positions(msg: JointState, joint_names: List[str]) -> Tuple[List[str], List[float]]:
    name_to_pos: Dict[str, float] = {n: p for n, p in zip(msg.name, msg.position)}
    missing = [n for n in joint_names if n not in name_to_pos]
    if missing:
        raise RuntimeError(f"/joint_states 缺少关节: {missing}. 实际关节名: {msg.name}")
    return joint_names, [name_to_pos[n] for n in joint_names]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "发送一条非常小的 FollowJointTrajectory 目标给 UR（会让机器人轻微运动）。\n"
            "默认不会执行，必须显式确认。"
        )
    )
    parser.add_argument("--robot", action="store_true", help="确认这将让真实机器人运动")
    parser.add_argument(
        "--action",
        default="/scaled_joint_trajectory_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action 名称",
    )
    parser.add_argument(
        "--joint",
        default="shoulder_pan_joint",
        choices=UR_JOINT_ORDER,
        help="要微动的关节",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=0.05,
        help="目标关节增量（弧度，默认 0.05rad 约 2.9°）",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="运动时间（秒）",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="等待 joint_states/action server 的超时（秒）",
    )
    args = parser.parse_args()

    if not args.robot:
        print(
            "拒绝执行：该脚本会让机器人运动。\n"
            "请确保：工作空间清空、速度滑块调低、急停可用，并且示教器 External Control 程序已运行。\n"
            "确认无误后用：--robot",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    try:
        js_node = OneShotJointState()
        js = js_node.wait(timeout_s=args.timeout)
        js_node.destroy_node()
        if js is None:
            print("超时：未收到 /joint_states。请确认驱动已启动且 joint_state_broadcaster active。", file=sys.stderr)
            return 1

        joint_names, current = _reorder_positions(js, UR_JOINT_ORDER)
        idx = joint_names.index(args.joint)
        target = list(current)
        target[idx] = target[idx] + args.delta

        node = Node("ur_send_small_trajectory")
        client = ActionClient(node, FollowJointTrajectory, args.action)

        if not client.wait_for_server(timeout_sec=args.timeout):
            print(
                f"超时：action server 不存在: {args.action}\n"
                "通常原因：控制器未激活。可先执行：ros2 control list_controllers\n"
                "并激活 scaled_joint_trajectory_controller。",
                file=sys.stderr,
            )
            node.destroy_node()
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        p0 = JointTrajectoryPoint()
        p0.positions = current
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0

        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.time_from_start.sec = int(args.duration)
        p1.time_from_start.nanosec = int((args.duration - int(args.duration)) * 1e9)

        goal.trajectory.points = [p0, p1]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("目标被拒绝（controller 可能 inactive、机器人程序未运行或处于受限状态）。", file=sys.stderr)
            node.destroy_node()
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout + args.duration + 2.0)
        res = result_future.result()
        if res is None:
            print("等待结果超时。", file=sys.stderr)
            node.destroy_node()
            return 1

        print(f"完成：result.error_code={res.result.error_code}")
        node.destroy_node()
        return 0
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
