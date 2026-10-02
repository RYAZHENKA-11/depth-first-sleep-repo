import dataclasses
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from go2_controller.core.geometry import CourseFrame, Pose, yaw_from_quaternion
from go2_controller.core.local_planner import PlannerParams
from go2_controller.core.mission import ABORT, ARRIVED, IDLE, Mission, MissionParams
from go2_controller.core.scan_utils import make_scan

INACTIVE = (IDLE, ARRIVED, ABORT)


def _declare_dataclass(node, prefix, cls):
    values = {}
    dyn = ParameterDescriptor(dynamic_typing=True)
    for f in dataclasses.fields(cls):
        default = f.default
        value = node.declare_parameter(f"{prefix}.{f.name}", default, dyn).value
        values[f.name] = type(default)(value)
    return cls(**values)


class ControllerNode(Node):

    def __init__(self):
        super().__init__("go2_controller")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("data_timeout_s", 0.5)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("scan_frame", "base_link")
        self.declare_parameter("odom_jump_warn_m", 0.3)

        mission_params = _declare_dataclass(self, "mission", MissionParams)
        planner_params = _declare_dataclass(self, "planner", PlannerParams)
        self.mission = Mission(mission_params, planner_params)

        self._timeout = float(self.get_parameter("data_timeout_s").value)
        self._frame = self.get_parameter("frame_id").value
        self._scan_frame = self.get_parameter("scan_frame").value
        self._jump_warn = float(self.get_parameter("odom_jump_warn_m").value)
        cmd_topic = self.get_parameter("cmd_vel_topic").value
        if self.get_parameter("dry_run").value:
            cmd_topic = "~/cmd_vel_dry"

        self._pose = None
        self._pose_t = None
        self._scan = None
        self._scan_t = None
        self._was_active = False
        self._last_state = None

        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_state = self.create_publisher(String, "~/state", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "~/markers", 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value,
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter("scan_topic").value,
                                 self._on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)

        if self.get_parameter("auto_start").value:
            self.mission.start()
        self.get_logger().info(
            f"go2_controller: cmd -> {self.pub_cmd.topic_name}, "
            f"auto_start={self.get_parameter('auto_start').value}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        pose = Pose(msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self._pose is not None:
            jump = math.hypot(pose.x - self._pose.x, pose.y - self._pose.y)
            if jump > self._jump_warn:
                self.get_logger().warning(
                    f"/odom jumped by {jump:.2f} m between messages: start pose and goals "
                    f"are fixed in the odom frame, relocalization breaks the mission",
                    throttle_duration_sec=5.0)
        self._pose = pose
        self._pose_t = self._now()

    def _on_scan(self, msg):
        if msg.header.frame_id != self._scan_frame:
            self.get_logger().warning(
                f"scan frame_id is '{msg.header.frame_id}', expected '{self._scan_frame}': "
                f"ranges are interpreted from the robot center, obstacles will be misplaced",
                throttle_duration_sec=5.0)
        self._scan = make_scan(np.asarray(msg.ranges, dtype=float), msg.angle_min,
                               msg.angle_increment, msg.range_min, msg.range_max)
        self._scan_t = self._now()

    def _srv_start(self, request, response):
        if self.mission.state not in INACTIVE:
            response.success = False
            response.message = f"mission already running: {self.mission.state}"
            return response
        self.mission.start()
        response.success = True
        response.message = "mission armed"
        self.get_logger().info("mission armed")
        return response

    def _srv_stop(self, request, response):
        self.mission.stop("stop service")
        self._publish_cmd(0.0, 0.0)
        response.success = True
        response.message = "mission stopped"
        self.get_logger().warning("mission stopped by service")
        return response

    def _fresh(self, stamp, now):
        return stamp is not None and now - stamp <= self._timeout

    def _tick(self):
        now = self._now()
        if not (self._fresh(self._pose_t, now) and self._fresh(self._scan_t, now)):
            if self.mission.state not in INACTIVE:
                self._publish_cmd(0.0, 0.0)
            self._publish_state({"state": self.mission.state, "waiting_for_data": True,
                                 "odom_ok": self._fresh(self._pose_t, now),
                                 "scan_ok": self._fresh(self._scan_t, now)})
            return

        out = self.mission.step(now, self._pose, self._scan)
        active = out.state not in INACTIVE
        if active:
            self._publish_cmd(out.vx, out.wz)
        elif self._was_active:
            self._publish_cmd(0.0, 0.0)
        self._was_active = active

        if out.state != self._last_state:
            self.get_logger().info(f"state -> {out.state} {out.info.get('reason', '')}")
            self._last_state = out.state
        info = dict(out.info)
        info.update(vx=round(out.vx, 3), wz=round(out.wz, 3))
        self._publish_state(info)
        self._publish_markers(info)

    def _publish_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def _publish_state(self, info):
        self.pub_state.publish(String(data=json.dumps(info)))

    def _publish_markers(self, info):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        start = info.get("start")
        if start is not None:
            course = CourseFrame(Pose(*start))
            arr.markers.append(self._marker(0, Marker.ARROW, stamp, (0.1, 0.8, 0.2),
                                            pose=start, scale=(0.6, 0.08, 0.08)))
            far = self.mission.p.outbound_max_dist
            line = [course.to_world(0.0, 0.0), course.to_world(far, 0.0)]
            arr.markers.append(self._marker(1, Marker.LINE_STRIP, stamp, (0.6, 0.6, 0.6),
                                            points=line, scale=(0.02, 0.0, 0.0)))
            finish = self._marker(2, Marker.CYLINDER, stamp, (0.1, 0.8, 0.2),
                                  pose=start, scale=(2 * self.mission.p.finish_tol,) * 2 + (0.01,))
            finish.color.a = 0.3
            arr.markers.append(finish)
            if "obstacle_s" in info:
                w = self.mission.p.detect_half_width
                for i, key in ((3, "obstacle_s"), (4, "obstacle_far_s")):
                    s = info[key]
                    pts = [course.to_world(s, -w), course.to_world(s, w)]
                    arr.markers.append(self._marker(i, Marker.LINE_STRIP, stamp, (1.0, 0.3, 0.1),
                                                    points=pts, scale=(0.04, 0.0, 0.0)))
        goal = info.get("goal")
        m = self._marker(5, Marker.SPHERE, stamp, (0.95, 0.85, 0.1),
                         pose=(goal[0], goal[1], 0.0) if goal else (0.0, 0.0, 0.0),
                         scale=(0.2, 0.2, 0.2))
        if goal is None:
            m.action = Marker.DELETE
        arr.markers.append(m)
        if self._pose is not None:
            text = self._marker(6, Marker.TEXT_VIEW_FACING, stamp, (1.0, 1.0, 1.0),
                                pose=(self._pose.x, self._pose.y, 0.0), scale=(0.0, 0.0, 0.18))
            text.pose.position.z = 0.85
            reason = info.get("reason") or ""
            text.text = info.get("state", "") + (f"\n{reason}" if reason else "")
            arr.markers.append(text)
        self.pub_markers.publish(arr)

    def _marker(self, mid, mtype, stamp, rgb, pose=None, points=None, scale=(0.1, 0.1, 0.1)):
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = stamp
        m.ns = "mission"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        if pose is not None:
            m.pose.position.x, m.pose.position.y = float(pose[0]), float(pose[1])
            m.pose.position.z = 0.02
            m.pose.orientation.z = math.sin(pose[2] / 2.0)
            m.pose.orientation.w = math.cos(pose[2] / 2.0)
        if points is not None:
            m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in points]
        return m

    def stop_robot(self):
        if self.mission.state not in INACTIVE or self._was_active:
            self._publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
