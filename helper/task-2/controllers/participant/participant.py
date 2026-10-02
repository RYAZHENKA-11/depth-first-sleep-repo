import math
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "common", "locomotion"
    ),
)

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from go2_api import Go2
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

SHOW_EVERY_N_STEPS: int = 100

LIDAR_X: float = 0.2
LIDAR_Z: float = 0.16
ROBOT_HALF_L: float = 0.5
ROBOT_HALF_W: float = 0.3

LIDAR_MIN_H: float = 0.25

GOAL_X: float = 31.0
GOAL_Y: float = -0.75
GOAL_RESEND_INTERVAL: float = 2.0

FULL_MAP_ORIGIN_X: float = 0.0
FULL_MAP_ORIGIN_Y: float = -13.0
FULL_MAP_WIDTH: float = 36.0
FULL_MAP_HEIGHT: float = 30.0

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
SLAM_LAUNCH = os.path.join(HERE, "slam_launch.py")
NAV2_LAUNCH = os.path.join(HERE, "nav2_launch.py")
DOCKER_DIR = os.path.join(PROJECT_ROOT, "docker")
COMPOSE_FILE = os.path.join(DOCKER_DIR, "docker-compose.yml")

_LIDAR_GEOM_CACHE: dict[tuple[float, int, int, float], tuple[float, float, float]] = {}


def _docker_up(
    service: str, label: str
) -> tuple[subprocess.Popen, str] | tuple[None, None]:
    try:
        proc = subprocess.Popen(
            ["docker", "compose", "-f", COMPOSE_FILE, "up", "-d", service],
            cwd=DOCKER_DIR,
        )
        proc.wait(timeout=30.0)
        print(f"[participant] {label}: docker service '{service}' started")
        return proc, "docker"
    except Exception as e:  # noqa: BLE001
        print(f"[participant] {label}: docker service '{service}' failed: {e}")
        return None, None


def _docker_stop(service: str, label: str) -> None:
    try:
        subprocess.Popen(
            ["docker", "compose", "-f", COMPOSE_FILE, "down", service],
            cwd=DOCKER_DIR,
        ).wait(timeout=15.0)
        print(f"[participant] {label}: docker service '{service}' stopped")
    except Exception as e:  # noqa: BLE001
        print(f"[participant] {label}: docker service '{service}' stop failed: {e}")


def _launch_ros2(
    launch_file: str, service: str, label: str
) -> tuple[subprocess.Popen, str] | tuple[None, None]:
    ros2 = shutil.which("ros2")
    if ros2 is not None:
        cmd = [ros2, "launch", launch_file, "use_sim_time:=false"]
        try:
            proc = subprocess.Popen(cmd)
            time.sleep(5.0)
            if proc.poll() is None:
                print(f"[participant] {label}: launched locally (pid {proc.pid})")
                return proc, "local"
            print(f"[participant] {label}: local launch exited early, trying docker...")
        except Exception as e:  # noqa: BLE001
            print(f"[participant] {label}: local launch failed: {e}")
    print(f"[participant] {label}: ros2 not found on host, trying docker...")
    return _docker_up(service, label)


def start_slam() -> subprocess.Popen | str | None:
    return _launch_ros2(SLAM_LAUNCH, "slam", "SLAM")


def stop_slam(proc: subprocess.Popen | None, mode: str | None) -> None:
    if proc is None:
        return
    if mode == "local":
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
    elif mode == "docker":
        _docker_stop("slam", "SLAM")


def start_nav2():
    return _launch_ros2(NAV2_LAUNCH, "nav2", "NAV2")


def stop_nav2(proc, mode) -> None:
    if mode == "local":
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
    else:
        _docker_stop("nav2", "NAV2")


class ParticipantNode(Node):
    def __init__(self) -> None:
        super().__init__("participant")

        self.cmd_vel = {"vx": 0.0, "vyaw": 0.0, "stamp": 0.0}

        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            "map_full",
            QoSProfile(
                depth=2,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            OccupancyGrid,
            "map",
            self._on_map,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self.get_logger().info("participant node started")

    def _on_cmd_vel(self, msg) -> None:
        self.cmd_vel["vx"] = msg.linear.x
        self.cmd_vel["vyaw"] = msg.angular.z
        self.cmd_vel["stamp"] = time.monotonic()

    def _on_map(self, msg: OccupancyGrid) -> None:
        res = msg.info.resolution
        src_w, src_h = msg.info.width, msg.info.height
        dst_w = round(FULL_MAP_WIDTH / res)
        dst_h = round(FULL_MAP_HEIGHT / res)
        off_x = round((msg.info.origin.position.x - FULL_MAP_ORIGIN_X) / res)
        off_y = round((msg.info.origin.position.y - FULL_MAP_ORIGIN_Y) / res)

        src = np.asarray(msg.data, dtype=np.int8).reshape(src_h, src_w)
        dst = np.full((dst_h, dst_w), -1, dtype=np.int8)

        sx0, sy0 = max(0, -off_x), max(0, -off_y)
        sx1 = min(src_w, dst_w - off_x)
        sy1 = min(src_h, dst_h - off_y)
        if sx1 > sx0 and sy1 > sy0:
            dst[sy0 + off_y : sy1 + off_y, sx0 + off_x : sx1 + off_x] = src[
                sy0:sy1, sx0:sx1
            ]

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
        full.data = dst.reshape(-1).tolist()
        self.map_pub.publish(full)

    def send_goal(self, x: float, y: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self.nav_client.send_goal_async(goal)


def obstacle_scan(robot) -> list[float]:
    def _unit_vectors(
        h_fov: float, h_res: int, layers: int, v_fov: float
    ) -> tuple[float, float, float]:
        key = (h_fov, h_res, layers, v_fov)

        cached = _LIDAR_GEOM_CACHE.get(key)
        if cached is not None:
            return cached

        azimuth, elevation = np.meshgrid(
            np.linspace(h_fov * -0.5, h_fov * 0.5, h_res),
            np.linspace(v_fov * 0.5, v_fov * -0.5, layers),
        )
        cos_e = np.cos(elevation)
        _LIDAR_GEOM_CACHE[key] = (
            cos_e * np.cos(azimuth),
            -cos_e * np.sin(azimuth),
            np.sin(elevation),
        )
        return _LIDAR_GEOM_CACHE[key]

    h_fov, h_res, max_range, layers, v_fov = robot.lidar_info()
    ranges = np.asarray(robot.lidar()).reshape(layers, h_res)
    dx, dy, dz = _unit_vectors(h_fov, h_res, layers, v_fov)

    roll, pitch, _yaw = robot.imu()
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    t = sr * dy + cr * dz
    ux = cp * dx + sp * t
    uy = cr * dy - sr * dz
    uz = -sp * dx + cp * t

    valid = np.isfinite(ranges)
    ranges = np.where(valid, ranges, 0.0)

    valid &= (
        robot.position()[2] + (-sp * LIDAR_X + cp * cr * LIDAR_Z) + ranges * uz
        >= LIDAR_MIN_H
    )
    ranges = ranges[valid]
    ux = ux[valid]
    uy = uy[valid]

    x = LIDAR_X + ranges * ux
    y = ranges * uy
    outside = (np.abs(x) >= ROBOT_HALF_L) | (np.abs(y) >= ROBOT_HALF_W)
    x = x[outside] - LIDAR_X
    y = y[outside]

    bearing = (np.arctan2(y, x) + math.pi) % (2.0 * math.pi) - math.pi
    angle_min = -0.5 * h_fov
    angle_inc = h_fov / (h_res - 1)
    idx = np.clip(
        np.rint((bearing - angle_min) / angle_inc).astype(np.int64), 0, h_res - 1
    )

    out = np.full(h_res, max_range)
    np.minimum.at(out, idx, np.hypot(x, y))
    return out.tolist()


def main() -> None:
    robot = Go2()

    rclpy.init()
    node = ParticipantNode()
    slam_proc, slam_mode = start_slam()
    nav2_proc, nav2_mode = start_nav2()

    h_fov, h_res, max_range, _layers, _v_fov = robot.lidar_info()
    angle_min = -h_fov * 0.5
    angle_max = h_fov * 0.5
    angle_inc = h_fov / max(h_res - 1, 1)

    scan_msg = LaserScan()
    scan_msg.header.frame_id = "laser"
    scan_msg.angle_min = angle_min
    scan_msg.angle_max = angle_max
    scan_msg.angle_increment = angle_inc
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1
    scan_msg.range_min = 0.1
    scan_msg.range_max = max_range

    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    step = 0
    last_goal_t = -GOAL_RESEND_INTERVAL
    while robot.step():
        rclpy.spin_once(node, timeout_sec=0.0)
        now = node.get_clock().now().to_msg()
        sim_t = robot.time()

        scan_msg.header.stamp = now
        scan_msg.ranges = obstacle_scan(robot)
        node.scan_pub.publish(scan_msg)

        x, y, yaw = robot.pose()
        odom_msg.header.stamp = now
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        node.odom_pub.publish(odom_msg)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = math.sin(yaw * 0.5)
        tf.transform.rotation.w = math.cos(yaw * 0.5)
        node.tf_broadcaster.sendTransform(tf)

        tf_laser = TransformStamped()
        tf_laser.header.stamp = now
        tf_laser.header.frame_id = "base_link"
        tf_laser.child_frame_id = "laser"
        tf_laser.transform.translation.x = LIDAR_X
        tf_laser.transform.translation.z = LIDAR_Z
        node.tf_broadcaster.sendTransform(tf_laser)

        robot.drive(node.cmd_vel["vx"], node.cmd_vel["vyaw"])
        if (sim_t - last_goal_t) >= GOAL_RESEND_INTERVAL:
            node.send_goal(GOAL_X, GOAL_Y)
            last_goal_t = sim_t

        step += 1
        if step % SHOW_EVERY_N_STEPS == 0:
            node.get_logger().info(
                f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, vx={node.cmd_vel['vx']:.2f}, vyaw={node.cmd_vel['vyaw']:.2f}"
            )

        # WARNING: participant: Forced termination (because process didn't terminate itself after 1 second).
        if robot.position()[2] > 2.5:
            stop_nav2(nav2_proc, nav2_mode)
            stop_slam(slam_proc, slam_mode)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
