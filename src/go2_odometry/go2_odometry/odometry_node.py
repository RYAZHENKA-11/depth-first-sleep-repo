import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

from go2_odometry.scan import cloud_to_scan, quaternion_to_yaw


class OdometryNode(Node):

    def __init__(self):
        super().__init__("go2_odometry")
        self.declare_parameter("pose_topic", "/go2/odom/robot_pose")
        self.declare_parameter("twist_topic", "/go2/odom/sport_lf")
        self.declare_parameter("imu_topic", "/go2/imu/data")
        self.declare_parameter("cloud_topic", "/go2/lidar/points")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("data_timeout_s", 0.5)
        self.declare_parameter("scan_rate_hz", 10.0)
        self.declare_parameter("beam_count", 360)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("min_obstacle_height", 0.08)
        self.declare_parameter("max_obstacle_height", 1.5)
        self.declare_parameter("body_half_length", 0.40)
        self.declare_parameter("body_half_width", 0.205)
        self.declare_parameter("body_height", 0.65)
        self.declare_parameter("odom_jump_warn_m", 0.3)

        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._timeout = float(self.get_parameter("data_timeout_s").value)
        scan_rate_hz = float(self.get_parameter("scan_rate_hz").value)
        if scan_rate_hz <= 0.0:
            raise ValueError("scan_rate_hz must be positive")
        self._beam_count = int(self.get_parameter("beam_count").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._scan_options = {
            "beam_count": self._beam_count,
            "range_min": self._range_min,
            "range_max": self._range_max,
            "min_obstacle_height": float(
                self.get_parameter("min_obstacle_height").value),
            "max_obstacle_height": float(
                self.get_parameter("max_obstacle_height").value),
            "body_half_length": float(self.get_parameter("body_half_length").value),
            "body_half_width": float(self.get_parameter("body_half_width").value),
            "body_height": float(self.get_parameter("body_height").value),
        }
        self._jump_warn = float(self.get_parameter("odom_jump_warn_m").value)
        self._pose = None
        self._pose_stamp = None
        self._twist = None
        self._twist_stamp = None
        self._imu = None
        self._imu_stamp = None
        self._cloud = None
        self._cloud_stamp = None

        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 10)
        self._scan_pub = self.create_publisher(
            LaserScan, self.get_parameter("scan_topic").value, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter("pose_topic").value,
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.get_parameter("twist_topic").value,
            self._on_twist, qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value,
            self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, self.get_parameter("cloud_topic").value,
            self._on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0 / scan_rate_hz, self._publish_scan)
        self.get_logger().info(
            f"go2_odometry: /odom and TF {self._odom_frame}->{self._base_frame}; "
            f"scan={self._beam_count} beams at {scan_rate_hz:g} Hz "
            f"while source data is fresh")

    def _age(self, stamp):
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def _is_fresh(self, stamp):
        if stamp is None:
            return False
        age = self._age(stamp)
        return 0.0 <= age <= self._timeout

    def _on_pose(self, msg):
        if msg.header.frame_id != self._odom_frame:
            self.get_logger().warning(
                f"Ignoring pose in frame '{msg.header.frame_id}', "
                f"expected '{self._odom_frame}'")
            return
        if msg.child_frame_id and msg.child_frame_id != self._base_frame:
            self.get_logger().warning(
                f"Ignoring pose child frame '{msg.child_frame_id}', "
                f"expected '{self._base_frame}'")
            return
        if not self._is_fresh(msg.header.stamp):
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        values = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in values):
            return
        try:
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w)
        except ValueError:
            self.get_logger().warning("Ignoring pose with invalid orientation")
            return
        if self._pose is not None and self._pose_stamp is not None:
            if Time.from_msg(msg.header.stamp) <= Time.from_msg(self._pose_stamp):
                return
            previous = self._pose[0]
            jump = math.hypot(position.x - previous[0], position.y - previous[1])
            if jump > self._jump_warn:
                self.get_logger().warning(
                    f"/go2/odom/robot_pose jumped by {jump:.2f} m")

        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2)
        self._pose = ((position.x, position.y, position.z), yaw,
                      orientation.x / quaternion_norm,
                      orientation.y / quaternion_norm,
                      orientation.z / quaternion_norm,
                      orientation.w / quaternion_norm)
        self._pose_stamp = msg.header.stamp
        self._publish_odometry()

    def _on_twist(self, msg):
        if not self._is_fresh(msg.header.stamp):
            return
        if self._twist_stamp is not None and (
                Time.from_msg(msg.header.stamp) <= Time.from_msg(self._twist_stamp)):
            return
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if all(math.isfinite(value) for value in values):
            self._twist = values
            self._twist_stamp = msg.header.stamp

    def _on_imu(self, msg):
        if not self._is_fresh(msg.header.stamp):
            return
        if self._imu_stamp is not None and (
                Time.from_msg(msg.header.stamp) <= Time.from_msg(self._imu_stamp)):
            return

        orientation = msg.orientation
        if not all(math.isfinite(value) for value in (
                orientation.x, orientation.y, orientation.z, orientation.w)):
            return
        try:
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w)
        except ValueError:
            self.get_logger().warning("Ignoring IMU with invalid orientation")
            return
        quaternion_norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2)
        quaternion = (
            orientation.x / quaternion_norm,
            orientation.y / quaternion_norm,
            orientation.z / quaternion_norm,
            orientation.w / quaternion_norm,
        )
        self._imu = (yaw, quaternion)
        self._imu_stamp = msg.header.stamp

        if self._pose is None:
            self._pose = ((0.0, 0.0, 0.0), yaw, *quaternion)
            self._pose_stamp = msg.header.stamp
            self._publish_odometry()

    def _on_cloud(self, msg):
        if msg.header.frame_id != self._odom_frame:
            self.get_logger().warning(
                f"Ignoring cloud in frame '{msg.header.frame_id}', "
                f"expected '{self._odom_frame}'")
            return
        if not self._is_fresh(msg.header.stamp):
            return
        if self._cloud_stamp is not None and (
                Time.from_msg(msg.header.stamp) <= Time.from_msg(self._cloud_stamp)):
            return
        try:
            point_data = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            if isinstance(point_data, np.ndarray) and point_data.dtype.names:
                points = np.column_stack(
                    (point_data["x"], point_data["y"], point_data["z"]))
            else:
                points = np.asarray(list(point_data), dtype=float)
                if points.size == 0:
                    points = np.empty((0, 3), dtype=float)
                else:
                    points = points.reshape((-1, 3))
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(f"Ignoring malformed point cloud: {exc}")
            return
        self._cloud = points
        self._cloud_stamp = msg.header.stamp

    def _publish_odometry(self):
        if self._pose is None or not self._is_fresh(self._pose_stamp):
            return
        position, _, qx, qy, qz, qw = self._pose
        msg = Odometry()
        msg.header.stamp = self._pose_stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        if self._twist is not None and self._is_fresh(self._twist_stamp):
            (msg.twist.twist.linear.x,
             msg.twist.twist.linear.y,
             msg.twist.twist.angular.z) = self._twist

        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)
        self._odom_pub.publish(msg)

    def _publish_scan(self):
        if (self._pose is None or self._cloud is None
                or not self._is_fresh(self._pose_stamp)
                or not self._is_fresh(self._cloud_stamp)):
            return
        pose_cloud_delta = abs((Time.from_msg(self._pose_stamp)
                                - Time.from_msg(self._cloud_stamp)).nanoseconds) * 1e-9
        if pose_cloud_delta > self._timeout:
            return
        position = self._pose[0]
        orientation = self._pose[2:]
        try:
            ranges = cloud_to_scan(
                self._cloud, position, orientation, **self._scan_options)
        except (ValueError, FloatingPointError) as exc:
            self.get_logger().warning(f"Could not project point cloud: {exc}")
            return

        scan = LaserScan()
        scan.header.stamp = self._pose_stamp
        scan.header.frame_id = self._base_frame
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self._beam_count
        scan.angle_max = scan.angle_min + scan.angle_increment * (self._beam_count - 1)
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.scan_time = 1.0 / float(self.get_parameter("scan_rate_hz").value)
        scan.ranges = ranges.tolist()
        self._scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()