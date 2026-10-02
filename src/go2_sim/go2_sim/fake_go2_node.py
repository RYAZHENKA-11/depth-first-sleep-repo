import math

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from go2_sim.world import (
    GO2_HALF_LENGTH, GO2_HALF_WIDTH, DriftingOdometry, KinematicRobot, World,
    footprint_clearance, voxel_cloud)

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
MODES = ("contract", "bridge")
CLOUD_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate("xyz")]


class FakeGo2(Node):

    def __init__(self):
        super().__init__("fake_go2")
        self.declare_parameter("world_file", "")
        self.declare_parameter("physics_rate", 50.0)
        self.declare_parameter("scan_rate", 10.0)
        self.declare_parameter("tau", 0.25)
        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_wz", 1.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("scan_beams", 360)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("range_noise", 0.01)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("mode", "contract")
        self.declare_parameter("odom_rate", 20.0)
        self.declare_parameter("cloud_rate", 4.0)
        self.declare_parameter("cloud_floor", True)
        self.declare_parameter("odom_scale_error", 0.03)
        self.declare_parameter("odom_yaw_bias", 0.003)
        self.declare_parameter("pose_noise", 0.01)

        self._mode = self.get_parameter("mode").value
        if self._mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {self._mode!r}")
        self.world = self._load_world(self.get_parameter("world_file").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._cmd = (0.0, 0.0)
        self._cmd_t = -math.inf
        self._dt = 1.0 / float(self.get_parameter("physics_rate").value)
        self._rng = np.random.default_rng(0)
        self._collided = None
        self._path = Path()
        self._path.header.frame_id = self._odom_frame
        self._reset_robot()

        self.pub_obstacles = self.create_publisher(MarkerArray, "~/obstacles", LATCHED)
        self.pub_path = self.create_publisher(Path, "~/path", 10)
        self.pub_collision = self.create_publisher(Bool, "~/collision", LATCHED)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.create_timer(self._dt, self._physics)
        self.create_timer(0.2, self._publish_path)

        if self._mode == "contract":
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
            self.tf = TransformBroadcaster(self)
            self.create_timer(1.0 / float(self.get_parameter("scan_rate").value),
                              self._publish_scan)
        else:
            self.pub_sport = self.create_publisher(Odometry, "/go2/odom/sport_lf", 10)
            self.pub_robot_pose = self.create_publisher(Odometry, "/go2/odom/robot_pose", 10)
            self.pub_imu = self.create_publisher(Imu, "/go2/imu/data", 10)
            self.pub_cloud = self.create_publisher(PointCloud2, "/go2/lidar/points", 10)
            self.pub_truth = self.create_publisher(MarkerArray, "~/truth", 10)
            odom_period = 1.0 / float(self.get_parameter("odom_rate").value)
            self.create_timer(odom_period, self._publish_bridge_state)
            self.create_timer(1.0 / float(self.get_parameter("cloud_rate").value),
                              self._publish_cloud)
            self.create_timer(0.1, self._publish_truth)

        self._publish_obstacles()
        self.get_logger().info(
            f"fake_go2 [{self._mode}]: {len(self.world.circles)} circles, "
            f"{len(self.world.boxes)} boxes, start={tuple(self.world.start)}")

    def _load_world(self, path):
        if not path:
            return World()
        with open(path) as f:
            return World.from_dict(yaml.safe_load(f) or {})

    def _reset_robot(self):
        self.robot = KinematicRobot(self.world.start, tau=float(self.get_parameter("tau").value),
                                    max_vx=float(self.get_parameter("max_vx").value),
                                    max_wz=float(self.get_parameter("max_wz").value))
        self.leg_odom = DriftingOdometry(
            self.world.start, scale_error=float(self.get_parameter("odom_scale_error").value),
            yaw_bias=float(self.get_parameter("odom_yaw_bias").value))
        self._path.poses = []

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        vx, wz = msg.linear.x, msg.angular.z
        if math.isfinite(vx) and math.isfinite(wz):
            self._cmd = (vx, wz)
            self._cmd_t = self._now()

    def _srv_reset(self, request, response):
        self._reset_robot()
        self._cmd = (0.0, 0.0)
        response.success = True
        response.message = "robot reset to world start"
        return response

    def _physics(self):
        if self._now() - self._cmd_t > float(self.get_parameter("cmd_timeout").value):
            self._cmd = (0.0, 0.0)
        pose = self.robot.step(self._cmd[0], self._cmd[1], self._dt)
        self.leg_odom.update(self.robot.vx, self.robot.wz, self._dt)
        collided = footprint_clearance(self.world, pose) <= 0.0
        if collided != self._collided:
            self._collided = collided
            self.pub_collision.publish(Bool(data=collided))
            if collided:
                self.get_logger().error(f"COLLISION at x={pose.x:.2f} y={pose.y:.2f}")
        if self._mode == "contract":
            self._publish_odom(pose)

    def _odometry(self, stamp, pose, with_twist):
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self._odom_frame
        od.child_frame_id = self._base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = float(pose.x), float(pose.y)
        od.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        if with_twist:
            od.twist.twist.linear.x = self.robot.vx
            od.twist.twist.angular.z = self.robot.wz
        return od

    def _publish_bridge_state(self):
        stamp = self.get_clock().now().to_msg()
        self.pub_sport.publish(self._odometry(stamp, self.leg_odom.pose, True))

        noise = float(self.get_parameter("pose_noise").value)
        p = self.robot.pose
        if noise > 0.0:
            dx, dy, dyaw = self._rng.normal(0.0, [noise, noise, noise * 0.3])
            p = type(p)(p.x + dx, p.y + dy, p.yaw + dyaw)
        self.pub_robot_pose.publish(self._odometry(stamp, p, False))

        yaw = self.leg_odom.pose.yaw
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.z, imu.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        imu.angular_velocity.z = self.robot.wz + self.leg_odom.yaw_bias
        imu.linear_acceleration.z = 9.81
        self.pub_imu.publish(imu)

    def _publish_cloud(self):
        p = self.robot.pose
        pts = voxel_cloud(self.world, p.x, p.y,
                          floor=bool(self.get_parameter("cloud_floor").value))
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = CLOUD_FIELDS
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.pub_cloud.publish(msg)

    def _publish_truth(self):
        p = self.robot.pose
        arr = MarkerArray()
        body = self._obstacle(0, Marker.CUBE, (p.x, p.y, 0.3),
                              (2 * GO2_HALF_LENGTH, 2 * GO2_HALF_WIDTH, 0.12))
        body.ns = "truth"
        body.pose.orientation.z = math.sin(p.yaw / 2.0)
        body.pose.orientation.w = math.cos(p.yaw / 2.0)
        body.color.r, body.color.g, body.color.b, body.color.a = 0.3, 0.9, 1.0, 0.5
        arr.markers.append(body)
        self.pub_truth.publish(arr)

    def _publish_odom(self, pose):
        od = self._odometry(self.get_clock().now().to_msg(), pose, True)
        t = TransformStamped()
        t.header = od.header
        t.child_frame_id = od.child_frame_id
        t.transform.translation.x = od.pose.pose.position.x
        t.transform.translation.y = od.pose.pose.position.y
        t.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(t)
        self.pub_odom.publish(od)

    def _publish_scan(self):
        scan = self.world.raycast(self.robot.pose, n=int(self.get_parameter("scan_beams").value),
                                  range_max=float(self.get_parameter("range_max").value))
        ranges = scan.ranges
        noise = float(self.get_parameter("range_noise").value)
        if noise > 0.0:
            finite = np.isfinite(ranges)
            ranges = ranges.copy()
            ranges[finite] += self._rng.normal(0.0, noise, int(finite.sum()))
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.angle_min = scan.angle_min
        msg.angle_increment = scan.angle_increment
        msg.angle_max = scan.angle_min + scan.angle_increment * (ranges.size - 1)
        msg.range_min = scan.range_min
        msg.range_max = scan.range_max
        msg.scan_time = 1.0 / float(self.get_parameter("scan_rate").value)
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub_scan.publish(msg)

    def _publish_path(self):
        p = PoseStamped()
        p.header.frame_id = self._odom_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = self.robot.pose.x, self.robot.pose.y
        p.pose.orientation.w = 1.0
        self._path.poses.append(p)
        self._path.header.stamp = p.header.stamp
        self.pub_path.publish(self._path)

    def _publish_obstacles(self):
        arr = MarkerArray()
        mid = 0
        for cx, cy, r in self.world.circles:
            arr.markers.append(self._obstacle(mid, Marker.CYLINDER, (cx, cy, 0.4),
                                              (2 * r, 2 * r, 0.8)))
            mid += 1
        for x0, y0, x1, y1 in self.world.boxes:
            arr.markers.append(self._obstacle(mid, Marker.CUBE, ((x0 + x1) / 2, (y0 + y1) / 2, 0.4),
                                              (x1 - x0, y1 - y0, 0.8)))
            mid += 1
        self.pub_obstacles.publish(arr)

    def _obstacle(self, mid, mtype, xyz, size):
        m = Marker()
        m.header.frame_id = self._odom_frame
        m.ns = "obstacles"
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = map(float, size)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.45, 0.15, 0.9
        return m


def main(args=None):
    rclpy.init(args=args)
    node = FakeGo2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
