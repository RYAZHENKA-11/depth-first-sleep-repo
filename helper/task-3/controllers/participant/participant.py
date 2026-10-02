#!/usr/bin/env python3
"""Уровень 3 «Полоса препятствий» — точка входа.

Цикл: шаг симуляции -> команда от Nav2 (или от fallback, если ROS2 недоступен)
-> публикация сенсоров -> цель от goal_manager -> шаг автомата руки.

Координаты трассы не читаются из config/ и worlds/ — это обнуляет результат.
"""
import math
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "common", "locomotion"))
from go2_api import Go2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from nav.fallback import tick as fallback_tick
from nav.perception import YELLOW_RGB
from arm.key_task import KeyTask

BRINGUP_DEADLINE_S = 55.0
STUCK_DIST_M = 0.05
STUCK_TIMEOUT_S = 8.0


class NavStack:
    """Nav2 + slam_toolbox. Опционален: при отказе .ok=False и main() уходит на fallback."""

    def __init__(self):
        self.ok = False
        self.bridge = None
        self.procman = None
        self._rclpy = None
        self._ready = False
        self._bringup_t0 = None

        if not shutil.which("ros2"):
            print("[participant] ros2 не найден в PATH — сразу на fallback", flush=True)
            return
        try:
            import rclpy
            from ros.process_manager import ProcessManager
            from ros.bridge_node import Go2Bridge
        except Exception as e:
            print(f"[participant] ROS2-стек недоступен ({e}) — на fallback", flush=True)
            return

        self._rclpy = rclpy
        rclpy.init()
        self.bridge = Go2Bridge()
        # отдельными процессами: их колбэки в потоке robot.step() застопорят симуляцию
        self.procman = ProcessManager(log_dir=os.path.join(HERE, "logs"))
        cfg = os.path.join(HERE, "config")
        self.procman.launch_slam(os.path.join(cfg, "slam_launch.py"),
                                 os.path.join(cfg, "slam_toolbox_params.yaml"))
        self.procman.launch_nav2(os.path.join(cfg, "nav2_launch.py"),
                                 os.path.join(cfg, "nav2_params.yaml"))
        self.ok = True
        self._bringup_t0 = time.time()

    def spin(self):
        if self.ok:
            self._rclpy.spin_once(self.bridge, timeout_sec=0.0)

    def ready(self):
        if not self.ok:
            return False
        if self._ready:
            return True
        restarted = self.procman.restart_exited()
        if restarted:
            print(f"[participant] перезапуск упавших процессов: {', '.join(restarted)}",
                  flush=True)
            self._bringup_t0 = time.time()
        if time.time() - self._bringup_t0 > BRINGUP_DEADLINE_S:
            print("[participant] bringup не уложился в дедлайн — на fallback", flush=True)
            self.ok = False
            return False
        self._ready = self.bridge.server_ready()
        return self._ready

    def shutdown(self):
        if self.procman is not None:
            self.procman.shutdown()
        if self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass


def main():
    robot = Go2()
    nav = NavStack()
    key_task = KeyTask() if getattr(robot.arm, "ok", False) else None

    goal_mgr = None
    if nav.ok:
        from nav.goal_manager import GoalManager
        goal_mgr = GoalManager(nav.bridge)

    using_nav = False
    last_pose = None
    last_move_t = 0.0

    try:
        while robot.step():
            now = robot.time()
            nav.spin()

            if nav.ok and nav.ready():
                if not using_nav:
                    print(f"[participant] Nav2 готов ({now:.1f}s) — переход на ROS2/Nav2", flush=True)
                using_nav = True
                vx, vyaw = nav.bridge.latest_cmd_vel()
                robot.drive(vx=vx, vyaw=vyaw)
                nav.bridge.publish_odom_tf(robot)
                nav.bridge.publish_scan_throttled(robot)
                nav.bridge.publish_camera_throttled(robot)
                goal_mgr.tick(robot, now)

                x, y, _yaw = robot.pose()
                if last_pose is None or math.hypot(x - last_pose[0], y - last_pose[1]) > STUCK_DIST_M:
                    last_pose = (x, y)
                    last_move_t = now
                elif now - last_move_t > STUCK_TIMEOUT_S:
                    nav.bridge.cancel_goal()
                    last_move_t = now

            elif nav.ok:
                nav.bridge.publish_odom_tf(robot)
                nav.bridge.publish_scan_throttled(robot)
                if goal_mgr is not None:
                    goal_mgr.observe(robot)
                stage_color = goal_mgr.target_color() if goal_mgr else YELLOW_RGB
                fallback_tick(robot, stage_color)

            else:
                if using_nav:
                    print("[participant] переключение на fallback-контроллер", flush=True)
                    using_nav = False
                stage_color = goal_mgr.target_color() if goal_mgr else YELLOW_RGB
                fallback_tick(robot, stage_color)

            if key_task is not None and not key_task.done:
                key_task.tick(robot)
    finally:
        nav.shutdown()


if __name__ == "__main__":
    main()
