"""Поднимает узлы Nav2 напрямую через lifecycle-сервисы, без nav2_lifecycle_manager.

Штатный менеджер держит с узлами bond и по таймауту heartbeat сносит весь стек
("CRITICAL FAILURE: SERVER ... IS DOWN"). В Webots на --mode=fast узел легко
пропускает heartbeat, просто ожидая TF, и навигация умирает на ровном месте.
Здесь bond'а нет: узлы переводятся в ACTIVE и переактивируются, если выпали.
"""
import time

import rclpy
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

MANAGED = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


class Bringup(Node):
    def __init__(self):
        super().__init__("nav2_bringup_supervisor")
        self._change = {n: self.create_client(ChangeState, f"/{n}/change_state") for n in MANAGED}
        self._get = {n: self.create_client(GetState, f"/{n}/get_state") for n in MANAGED}

    def _state(self, name):
        cli = self._get[name]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=2.0):
            return -1
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.done() and fut.result() is not None:
            return fut.result().current_state.id
        return -1

    def _transition(self, name, tid):
        cli = self._change[name]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=2.0):
            return False
        req = ChangeState.Request()
        req.transition.id = tid
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.done() and fut.result() is not None and fut.result().success

    def ensure_active(self):
        all_active = True
        for name in MANAGED:
            state = self._state(name)
            if state == LifecycleState.PRIMARY_STATE_ACTIVE:
                continue
            all_active = False
            if state == LifecycleState.PRIMARY_STATE_INACTIVE:
                tid = Transition.TRANSITION_ACTIVATE
            elif state in (LifecycleState.PRIMARY_STATE_UNCONFIGURED, -1):
                tid = Transition.TRANSITION_CONFIGURE
            else:
                continue
            if not self._transition(name, tid):
                self.get_logger().warn(f"{name}: переход {tid} не удался, повтор")
        return all_active


def main():
    rclpy.init()
    node = Bringup()
    was_active = False
    while rclpy.ok():
        active = node.ensure_active()
        if active != was_active:
            node.get_logger().info("все узлы Nav2 активны" if active
                                   else "узлы Nav2 выпали из ACTIVE, поднимаю заново")
        was_active = active
        time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
