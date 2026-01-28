"""灵巧手控制模块：qbSoftHand 控制"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from qbsofthand_control.srv import SetClosure


class HandController:
    """qbSoftHand 灵巧手控制器"""

    def __init__(self, node: Node, config: dict, callback_group=None):
        self.node = node
        self.config = config
        self.logger = node.get_logger()
        self.callback_group = callback_group

        # 配置参数
        hand_cfg = config.get('hand', {})
        service_name = hand_cfg.get(
            'service',
            '/qbsofthand_control_node/set_closure'
        )
        self.open_closure = hand_cfg.get('open_closure', 0.0)
        self.close_closure = hand_cfg.get('close_closure', 0.8)
        self.default_duration = hand_cfg.get('duration', 1.5)

        # Service 客户端（使用回调组）
        self.client = node.create_client(
            SetClosure, service_name,
            callback_group=callback_group
        )

        self.logger.info(f'灵巧手控制器初始化完成，Service: {service_name}')

    def wait_for_service(self, timeout_sec: float = 10.0) -> bool:
        """等待服务就绪"""
        self.logger.info('等待灵巧手服务...')
        ready = self.client.wait_for_service(timeout_sec=timeout_sec)
        if ready:
            self.logger.info('灵巧手服务已就绪')
        else:
            self.logger.error('灵巧手服务连接超时')
        return ready

    def set_closure(
        self,
        closure: float,
        duration_sec: float = None,
        speed_ratio: float = 1.0
    ) -> bool:
        """
        设置灵巧手闭合度

        Args:
            closure: 闭合度 [0.0, 1.0]，0=张开，1=闭合
            duration_sec: 持续时间（秒），>0 使用时间模式
            speed_ratio: 速度比例（duration_sec=0 时使用）

        Returns:
            是否成功
        """
        if duration_sec is None:
            duration_sec = self.default_duration

        # 限制范围
        closure = max(0.0, min(1.0, closure))

        request = SetClosure.Request()
        request.closure = float(closure)
        request.duration_sec = float(duration_sec)
        request.speed_ratio = float(speed_ratio)

        self.logger.info(
            f'设置灵巧手闭合度: {closure:.2f}, '
            f'时间: {duration_sec:.1f}s'
        )

        # 同步调用服务
        future = self.client.call_async(request)

        # 等待响应（使用简单循环，不使用 spin_until_future_complete）
        timeout = duration_sec + 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.logger.error('灵巧手控制超时')
                return False
            time.sleep(0.01)

        result = future.result()
        if result.success:
            self.logger.info('灵巧手控制成功')
            return True
        else:
            self.logger.error(f'灵巧手控制失败: {result.message}')
            return False

    def open_hand(self) -> bool:
        """张开灵巧手"""
        self.logger.info('张开灵巧手')
        return self.set_closure(self.open_closure)

    def close_hand(self) -> bool:
        """闭合灵巧手（抓取）"""
        self.logger.info('闭合灵巧手')
        return self.set_closure(self.close_closure)

    def is_ready(self) -> bool:
        """检查灵巧手服务是否就绪"""
        return self.client.service_is_ready()
