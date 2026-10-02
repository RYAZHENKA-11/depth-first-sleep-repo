"""Единственный rclpy-узел в процессе контроллера: /scan, /odom, TF, /cmd_vel и цели
Nav2. Крутится через spin_once(0.0) и не блокирует цикл robot.step().
"""
import array
import math
import os

import numpy as np
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from nav.fallback import LIDAR_X, LIDAR_Z, obstacle_scan

FULL_MAP_ORIGIN_X = -14.0
FULL_MAP_ORIGIN_Y = -14.0
FULL_MAP_WIDTH = 28.0
FULL_MAP_HEIGHT = 28.0

CMD_VEL_TIMEOUT_S = 0.5


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Go2Bridge(Node):
    def __init__(self):
        super().__init__("go2_bridge")
        self._cmd = (0.0, 0.0)
        self._cmd_t = -1e9
        self._scan_tick = 0

        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        self.tf_bc = TransformBroadcaster(self)
        self.static_tf_bc = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.pub_map_full = self.create_publisher(
            OccupancyGrid, "/map_full",
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._goal_handle = None
        self._nav_status = "idle"

        self._is_active_cli = self.create_client(GetState, "/bt_navigator/get_state")
        self._active_future = None
        self._lifecycle_active = False

        self.debug = os.environ.get("GO2_DEBUG") == "1"
        self._cam_tick = 0
        if self.debug:
            self.pub_goal = self.create_publisher(PoseStamped, "/go2/goal", 10)
            self.pub_cam = self.create_publisher(Image, "/go2/camera", 1)
            self.pub_markers = self.create_publisher(MarkerArray, "/go2/debug_markers", 10)

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.translation.x = LIDAR_X
        t.transform.translation.z = LIDAR_Z
        t.transform.rotation.w = 1.0
        self.static_tf_bc.sendTransform(t)

    def _on_map(self, msg):
        """Карта slam_toolbox -> фиксированный холст для статического слоя costmap."""
        res = msg.info.resolution
        if res <= 0.0:
            return
        src_w, src_h = msg.info.width, msg.info.height
        dst_w = round(FULL_MAP_WIDTH / res)
        dst_h = round(FULL_MAP_HEIGHT / res)
        off_x = round((msg.info.origin.position.x - FULL_MAP_ORIGIN_X) / res)
        off_y = round((msg.info.origin.position.y - FULL_MAP_ORIGIN_Y) / res)

        src = np.asarray(msg.data, dtype=np.int8).reshape(src_h, src_w)
        dst = np.full((dst_h, dst_w), -1, dtype=np.int8)
        sx0, sy0 = max(0, -off_x), max(0, -off_y)
        sx1, sy1 = min(src_w, dst_w - off_x), min(src_h, dst_h - off_y)
        if sx1 > sx0 and sy1 > sy0:
            dst[sy0 + off_y:sy1 + off_y, sx0 + off_x:sx1 + off_x] = src[sy0:sy1, sx0:sx1]

        full = OccupancyGrid()
        full.header.stamp = msg.header.stamp
        full.header.frame_id = "map"
        full.info.map_load_time = msg.info.map_load_time
        full.info.resolution = res
        full.info.width = dst_w
        full.info.height = dst_h
        full.info.origin.position.x = FULL_MAP_ORIGIN_X
        full.info.origin.position.y = FULL_MAP_ORIGIN_Y
        full.info.origin.orientation.w = 1.0
        full.data = array.array("b", dst.reshape(-1).tobytes())
        self.pub_map_full.publish(full)

    def _on_cmd_vel(self, msg):
        self._cmd = (msg.linear.x, msg.angular.z)
        self._cmd_t = self._now_s()

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def latest_cmd_vel(self):
        """Команда Nav2, пока она свежая: поток /cmd_vel обрывается вместе с целью."""
        if self._now_s() - self._cmd_t > CMD_VEL_TIMEOUT_S:
            return (0.0, 0.0)
        return self._cmd

    def server_ready(self):
        """ActionServer виден И bt_navigator в состоянии ACTIVE — иначе цели уходят в failed."""
        return self.nav_client.server_is_ready() and self._lifecycle_active_nonblocking()

    def _lifecycle_active_nonblocking(self):
        if self._lifecycle_active:
            return True
        if self._active_future is not None:
            if not self._active_future.done():
                return False
            try:
                res = self._active_future.result()
                self._lifecycle_active = (
                    res is not None
                    and res.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE)
            except Exception:
                self._lifecycle_active = False
            self._active_future = None
        elif self._is_active_cli.service_is_ready():
            self._active_future = self._is_active_cli.call_async(GetState.Request())
        return self._lifecycle_active

    def publish_odom_tf(self, robot):
        x, y, yaw = robot.pose()
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_to_quat(yaw)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_bc.sendTransform(t)

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.x, od.pose.pose.orientation.y = qx, qy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        self.pub_odom.publish(od)

    def publish_scan_throttled(self, robot, every_n_ticks=4):
        self._scan_tick += 1
        if self._scan_tick % every_n_ticks != 0:
            return
        ranges = obstacle_scan(robot)
        if not ranges:
            return
        h_fov, h_res, max_range, _layers, _v_fov = robot.lidar_info()
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = "laser"
        s.angle_min = -h_fov / 2.0
        s.angle_max = h_fov / 2.0
        s.angle_increment = h_fov / max(1, h_res - 1)
        s.range_min = 0.10
        s.range_max = float(max_range)
        # «пусто» = +inf: вместе с inf_is_valid в nav2_params costmap расчищает коридор
        s.ranges = [float(r) if r < max_range - 1e-6 else float("inf") for r in ranges]
        self.pub_scan.publish(s)

    def publish_camera_throttled(self, robot, every_n_ticks=25):
        if not self.debug:
            return
        self._cam_tick += 1
        if self._cam_tick % every_n_ticks != 0:
            return
        w, h, _fov = robot.camera_info()
        raw = robot.image()
        if not raw or w <= 0 or h <= 0 or len(raw) < w * h * 4:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height, msg.width = h, w
        msg.encoding = "bgra8"
        msg.is_bigendian = 0
        msg.step = w * 4
        msg.data = bytes(raw[: w * h * 4])
        self.pub_cam.publish(msg)

    def publish_marker_bearing(self, found, bearing, stage):
        if not self.debug:
            return
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "marker_bearing"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD if found else Marker.DELETE
        m.scale.x, m.scale.y, m.scale.z = 1.5, 0.06, 0.06
        m.color.a = 0.9
        m.color.r, m.color.g, m.color.b = (0.6, 0.2, 0.9) if stage >= 3 else (0.95, 0.85, 0.1)
        m.pose.position.z = 0.2
        m.pose.orientation.z = math.sin(bearing / 2.0)
        m.pose.orientation.w = math.cos(bearing / 2.0)
        arr.markers.append(m)
        self.pub_markers.publish(arr)

    def send_goal(self, x, y, frame_id="map"):
        if not self.nav_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        if self.debug:
            self.pub_goal.publish(goal.pose)
        self._nav_status = "active"
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        return True

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._nav_status = "failed"
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        # исход лежит в .status: future.result() не бросает исключение ни на ABORTED,
        # ни на CANCELED, и без этой проверки любой провал выглядел как успех
        try:
            status = future.result().status
        except Exception:
            self._nav_status = "failed"
            return
        self._nav_status = ("succeeded" if status == GoalStatus.STATUS_SUCCEEDED
                            else "failed")

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._nav_status = "failed"   # отменяем, когда застряли — цель надо менять

    def nav_status(self):
        return self._nav_status
