#!/usr/bin/env python3
# ROS 2 ↔ Go2 мост (через WebRTC). Даёт стандартные ROS-топики:
#   упр:  /go2/sport_cmd (std_msgs/String)  — sit/stand_up/hello/…
#         /cmd_vel       (geometry_msgs/Twist) — linear.x/y, angular.z → движение
#   сост: /joint_states  (sensor_msgs/JointState) — 12 углов суставов с робота
# Запуск: source /opt/ros/lyrical/setup.bash && ~/ai-robot/venv/bin/python ~/ai-robot/ros_bridge.py
import asyncio, os, sys, threading, time
sys.path.insert(0, os.path.expanduser("~/ai-robot"))
import go2
from unitree_webrtc_connect.constants import RTC_TOPIC
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

SPORT = {
    "sit": "Sit", "rise_sit": "RiseSit", "stand_up": "StandUp", "stand_down": "StandDown",
    "hello": "Hello", "stretch": "Stretch", "wiggle": "WiggleHips", "balance": "BalanceStand",
    "recovery": "RecoveryStand", "damp": "Damp", "stop": "StopMove",
}
JOINTS = ["FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
          "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"]
LIM = {"x": 0.6, "y": 0.5, "z": 1.0}

loop = asyncio.new_event_loop()
state = {"conn": None, "q": None}


def _q(m):
    return float(m["q"] if isinstance(m, dict) else getattr(m, "q", 0.0))


async def _setup():
    conn = await go2.connect()
    await go2.ensure_normal_mode(conn)

    def cb(msg):
        d = msg.get("data", msg) if isinstance(msg, dict) else msg
        ms = (d.get("motor_state") if isinstance(d, dict) else getattr(d, "motor_state", None)) or []
        if ms:
            state["q"] = [_q(m) for m in ms[:12]]

    conn.datachannel.pub_sub.subscribe(RTC_TOPIC.get("LOW_STATE", "rt/lf/lowstate"), cb)
    state["conn"] = conn


def _loop_thread():
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_setup())
    loop.run_forever()


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Go2Bridge(Node):
    def __init__(self):
        super().__init__("go2_bridge")
        self.create_subscription(String, "/go2/sport_cmd", self.on_sport, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_vel, 10)
        self.js = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self.pub_js)
        self.get_logger().info(
            "go2_bridge готов: /go2/sport_cmd + /cmd_vel → робот, /joint_states ← робот")

    def _run(self, coro):
        if not state["conn"]:
            self.get_logger().warning("WebRTC ещё не подключён")
            return
        try:
            asyncio.run_coroutine_threadsafe(coro, loop).result(timeout=10)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"ошибка: {e}")

    def on_sport(self, msg):
        name = SPORT.get((msg.data or "").strip())
        if not name:
            self.get_logger().warning(f"неизвестная команда: {msg.data!r}")
            return
        self.get_logger().info(f"sport → {name}")
        self._run(go2.sport(state["conn"], name))

    def on_vel(self, msg):
        x = _clamp(msg.linear.x, -LIM["x"], LIM["x"])
        y = _clamp(msg.linear.y, -LIM["y"], LIM["y"])
        z = _clamp(msg.angular.z, -LIM["z"], LIM["z"])
        self._run(go2.sport(state["conn"], "Move", {"x": x, "y": y, "z": z}))

    def pub_js(self):
        q = state["q"]
        if not q:
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = JOINTS
        m.position = [float(v) for v in q]
        self.js.publish(m)


def main():
    threading.Thread(target=_loop_thread, daemon=True).start()
    for _ in range(40):
        if state["conn"]:
            break
        time.sleep(0.5)
    if not state["conn"]:
        print("WebRTC не подключился — проверь ключ/связь с собакой", flush=True)
    rclpy.init()
    node = Go2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
