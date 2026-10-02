# go2_webrtc_bridge

Experimental ROS 2 bridge for Unitree Go2 using `unitree_webrtc_connect==2.1.2`.

## What this version bridges

### Telemetry / sensors

- `rt/lf/lowstate`
- `rt/lf/sportmodestate`
- `rt/sportmodestate`
- `rt/utlidar/voxel_map`
- `rt/utlidar/voxel_map_compressed`
- `rt/utlidar/lidar_state`
- `rt/utlidar/robot_pose`
- `rt/lio_sam_ros2/mapping/odometry`
- `rt/pctoimage_local`
- `rt/qt_notice`
- `rt/uslam/frontend/cloud_world_ds`
- `rt/uslam/frontend/odom`
- `rt/uslam/server_log`
- `rt/uslam/localization/cloud_world`
- `rt/uslam/localization/odom`
- `rt/mapping/grid_map`

### Commands

- `rt/utlidar/switch`
- `rt/uslam/client_command`
- `rt/qt_command`
- `rt/api/sport/request` via `/cmd_vel` (disabled by default)

## Typed ROS outputs

- `/go2/imu/data` -> `sensor_msgs/msg/Imu`
- `/go2/joint_states` -> `sensor_msgs/msg/JointState`
- `/go2/lidar/state` -> `std_msgs/msg/String` (typed JSON of `rt/utlidar/lidar_state`)
- `/go2/odom/sport_lf` -> `nav_msgs/msg/Odometry`
- `/go2/odom/sport` -> `nav_msgs/msg/Odometry`
- `/go2/odom/robot_pose` -> `nav_msgs/msg/Odometry`
- `/go2/odom/lio_sam` -> `nav_msgs/msg/Odometry` when WebRTC payload is JSON-like
- `/go2/odom/uslam_mapping` -> `nav_msgs/msg/Odometry` when JSON-like
- `/go2/odom/uslam_localization` -> `nav_msgs/msg/Odometry` when JSON-like
- `/go2/lidar/points` -> `sensor_msgs/msg/PointCloud2`
- `/go2/lidar/points_compressed` -> `sensor_msgs/msg/PointCloud2`

Unknown/non-JSON payloads are also preserved as raw ROS messages under
`/go2/raw/...`. Those channels carry the polled topics in full — see
[Recording fidelity](#recording-fidelity) for what that costs in disk.

### The bridge refuses rather than guesses

Three places deliberately publish **nothing** instead of publishing something
that looks plausible but is not. All three keep the payload on `/go2/raw/...`:

- Binary CDR on an odometry topic. There is no way to decode it, and an
  all-zero `Odometry` is indistinguishable from "the robot is at the origin".
- A JSON payload that contains none of the fields the odometry converter knows
  (`position`, `pose`, `quaternion`, `velocity`, `twist`, `yaw_speed`, ...). Same
  reason: `/go2/odom/lio_sam` and `/go2/odom/uslam_*` stay silent until
  localization is actually enabled and the payload is actually odometry.
- `/cmd_vel` carrying `NaN`/`inf`. `np.clip` propagates `NaN`, so an unguarded
  value would be transmitted to the robot as a `Move` parameter.

`nav_msgs/msg/Odometry` covariance is left at its default (all zeros) everywhere.
Zeros are the conventional "unknown" value; if you need a non-trivial covariance
you have to derive it yourself.

`/go2/raw/...` and `/go2/lidar/state` are always strict-valid JSON: `NaN` and
`Infinity` (which `json.dumps` emits by default, and which are *not* valid JSON
per RFC 8259) are emitted as `null`.

## Payload shapes, as recorded

Verified against the pickle dumps in `backup_code/` (`dump.pkl`, `dump2.pkl`,
`dump3.pkl`, `dump_w1.pkl`, `dump_w2.pkl`). Those are the only topics that ever
produced data; `SLAM_QT_NOTICE`, `LIDAR_LOCALIZATION_*` and `GRID_MAP` stayed
silent in every run.

```text
rt/lf/lowstate
  imu_state:  {rpy: [3]}                       <- no quaternion/gyro/accel
  motor_state: 20 x {q, temperature, lost, reserve}
                indices 0-11 = real leg joints, 12-19 = constant zero placeholders
                there is no "dq" anywhere, so JointState.velocity stays unset
  bms_state, foot_force, temperature_ntc1, power_v

rt/lf/sportmodestate
  imu_state:  {quaternion: [w,x,y,z], gyroscope: [3], accelerometer: [3],
               rpy: [3], temperature}
              <- the only complete sensor_msgs/Imu source
  position: [3]   body-frame velocity is `velocity` (not odom)
  velocity: [3]   yaw_speed is exactly gyroscope[2]
  stamp, mode, progress, gait_type, error_code, foot_* , range_obstacle

rt/utlidar/robot_pose        (ROBOTODOM)
  header: {stamp: {sec, nanosec}, frame_id: "odom"}, pose: {position, orientation}
              <- no twist/velocity at all, so /go2/odom/robot_pose twist is zero

rt/utlidar/voxel_map         (ULIDAR_ARRAY)
  stamp, frame_id: "odom", resolution: 0.05, src_size,
  origin: [3], width: [128,128,38],
  data: {points: float64 ndarray (N, 3)}      <- world grid, already in odom

rt/utlidar/lidar_state       (ULIDAR_STATE)
  flat dict; note imu_rpy here is DEGREES, unlike every other topic

rt/multiple_state            a JSON *string*, e.g. {"brightness":0,...}
```

`rt/lf/lowstate` and `rt/lf/sportmodestate` are the two sources of the robot's
IMU, and they do not carry the same fields. `imu_source` selects which one feeds
`/go2/imu/data`:

| value | source | orientation | angular_velocity | linear_acceleration |
| --- | --- | --- | --- | --- |
| `sport` (default) | `rt/lf/sportmodestate` | quaternion | gyroscope | accelerometer |
| `lowstate` | `rt/lf/lowstate` | derived from rpy | zero | zero |
| `both` | both, interleaved | quaternion / rpy | mixed | mixed |

`imu_source: sport` is the default because the alternative silently publishes
zeroed angular velocity and linear acceleration on a live `sensor_msgs/Imu`,
which looks valid but is not real data.

### Frames

`nav_msgs/Odometry` requires the twist in `child_frame_id`, not
`header.frame_id`. `SportModState.velocity` and `yaw_speed` are body-frame
vectors, so they are correct under a body `child_frame_id` with no rotation.
`ULIDAR_ARRAY.frame_id` is `"odom"` in every recorded cloud, so the cloud keeps
it (`lidar_use_payload_frame: true`) instead of being relabelled `lidar_link`.

## Activity monitor

`/go2/bridge/topic_status` is a JSON `std_msgs/msg/String` containing per-topic:

- count
- measured rate
- age since last message
- last payload size
- `active` (message received within last 2 s)

This is intended specifically for bring-up / finding which Go2 subsystems are actually live.

## uSLAM commands

Publish a command string:

```bash
ros2 topic pub --once /go2/uslam/command std_msgs/msg/String "{data: 'mapping/start'}"
```

Examples:

```text
mapping/start
mapping/stop
mapping/get_status
localization/get_status
navigation/get_status
common/get_map_id
localization/set_initial_pose/0.0/0.0/0.0
localization/start
localization/stop
```

For the legacy LIO-SAM/Qt path:

```bash
ros2 topic pub --once /go2/legacy_slam/command std_msgs/msg/String "{data: '...'}"
```

The bridge does not guess the legacy Qt command semantics.

## Sport commands and motion safety

`/go2/sport_cmd` (`std_msgs/msg/String`) maps a friendly alias onto a
`SPORT_CMD` api_id and sends it on `rt/api/sport/request`:

```bash
ros2 topic pub --once /go2/sport_cmd std_msgs/msg/String "{data: 'stand_up'}"
```

```text
sit  rise_sit  stand_up  stand_down  hello  stretch
wiggle  balance  recovery  damp  stop
```

The api_ids are resolved **once, at import**, and an alias whose
`SPORT_CMD` entry does not exist in the installed library is dropped with a
startup warning rather than raising `KeyError` at the moment you ask the robot
to sit. `Move` and `StopMove` are checked explicitly and are a hard startup error
if absent.

`enable_cmd_vel` is off by default. When it is on, `/cmd_vel` and
`/cmd_vel_stamped` are resampled onto the robot at 10 Hz, and three separate
things stop the robot:

- **Watchdog** (ROS timer, `cmd_vel_timeout`): no fresh `/cmd_vel` within the
  timeout clears the command and sends `StopMove`.
- **Shutdown** (`Ctrl-C`, `SIGINT`): `StopMove` is sent unconditionally whenever
  `enable_cmd_vel` is on, before the session is closed — not only if a command
  happens to be live at that instant.
- **Link loss**: the WebRTC watchdog attempts a `StopMove` before tearing the
  session down. The channel that carried the last `Move` is exactly the thing
  that just died, so this is best effort; it is one round trip well spent.

In all three paths the periodic `Move` resend is gated *before* the `StopMove`
is issued, so the last motion command on the wire is always `StopMove`.

## LiDAR

Enable it explicitly during bring-up:

```bash
ros2 service call /go2/lidar/set_enabled std_srvs/srv/SetBool "{data: true}"
```

Disable:

```bash
ros2 service call /go2/lidar/set_enabled std_srvs/srv/SetBool "{data: false}"
```

The v2.1.2 library turns off traffic saving, selects the configured decoder, then sends `rt/utlidar/switch = "on"`. Its LiDAR decoder produces a point array for the `utlidar/*` binary streams.

## Installation

Use the same Python environment that contains `unitree_webrtc_connect==2.1.2`.

From the workspace root:

```bash
cd ~/ros2_ws/src
git clone <your-copy-of-this-package> go2_webrtc_bridge
cd ~/ros2_ws
git clone ...  # only if you need to install your Python dependencies separately
source /path/to/venv/bin/activate
pip install -e /path/to/unitree_webrtc_connect==2.1.2
colcon build --symlink-install --packages-select go2_webrtc_bridge
source install/setup.bash
```

If ROS 2 is installed from apt and `unitree_webrtc_connect` lives in a separate venv, the simplest arrangement is a venv created with access to the system ROS Python packages, e.g. `python3 -m venv --system-site-packages ~/ai-robot/venv`. Then install `unitree_webrtc_connect==2.1.2` into that venv and launch the node with that interpreter. The bridge intentionally imports the WebRTC library at node startup.

## Run

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2_webrtc_bridge go2_webrtc_bridge.launch.py \
  robot_ip:=192.168.8.181 \
  aes_128_key:=YOUR_32_HEX_KEY
```

For initial data collection keep:

```text
lidar_auto_enable=false
enable_cmd_vel=false
```

Then turn LiDAR / uSLAM on explicitly and record:

```bash
ros2 bag record -a -o go2_experiment
```

### Recording fidelity

The bridge does not truncate the WebRTC topics it polls. Measured against the
dumps in `backup_code/`, the only place data used to be thrown away was the
utlidar voxel-map point array, which was replaced by a
`{"__ndarray__": true, "dtype": ..., "shape": ...}` stub — **215 bytes
recorded instead of 1.46 MB**, i.e. the entire LiDAR stream. `publish_full_arrays`
(now the default) writes the array out in full, as float64, with no precision
loss and no per-point rounding.

What that costs per hour, at the rates in the dumps, with the default settings
(`publish_full_arrays=true`, `exclude_raw_fields=["motor_state"]`):

| channel | rate | size/msg | per hour |
| --- | --- | --- | --- |
| `ULIDAR_ARRAY` (voxel map) | 2.9-5.1 Hz | ~1.46 MB | **15-25 GB** |
| `LF_SPORT_MOD_STATE` | 20 Hz | 0.6 KB | 43 MB |
| `LOW_STATE` | 20 Hz | 0.24 KB | 17 MB |
| `ROBOTODOM` | 19 Hz | 0.2 KB | 14 MB |
| `ULIDAR_STATE` | 5 Hz | 0.5 KB | 8 MB |
| everything else | — | — | <1 MB |

So a full-fidelity run is **~15-25 GB/hour, over 99% of it LiDAR**. Check free
disk before starting. If that is too much:

```bash
# ~0.09 GB/hour. The raw channel keeps every scalar field of the cloud
# (origin, resolution, width, src_size, stamp, frame_id) and stubs only the
# point array; /go2/lidar/points still carries every point.
ros2 launch go2_webrtc_bridge go2_webrtc_bridge.launch.py publish_full_arrays:=false
```

`/go2/lidar/points` (`sensor_msgs/msg/PointCloud2`) is the better channel for the
points themselves: ~420 KB per cloud instead of 1.46 MB, and it is the standard
bag representation. Recorded together with the raw channel nothing is lost — the
raw channel carries the voxel-map metadata (`origin`, `resolution`, `width`,
`src_size`, `frame_id`) that `PointCloud2` has no field for, and the raw channel
keeps float64 where `PointCloud2` stores float32.

#### Joint positions

`exclude_raw_fields` defaults to `["motor_state"]`, which drops the per-motor
arrays from `/go2/raw/rt_lf_lowstate`. `motor_state` is 1105 of that message's
1358 bytes — but `LOW_STATE` as a whole is under 0.2% of the traffic, so this
saves ~17 MB/hour out of ~20 GB/hour. It is worth doing because the data is not
useful to you, not because it is large. Positions remain complete on the typed
`/go2/joint_states` topic; `exclude_raw_fields:=` (empty) also restores them to
the raw channel.

#### A warning about the 1.46 MB messages

With `publish_full_arrays:=true`, `/go2/raw/rt_utlidar_voxel_map` carries a single
`std_msgs/msg/String` of ~1.46 MB at 3-5 Hz. This has not been tested against a
real DDS transport, and large `String` messages depend on the RMW's fragmentation
support and `max_message_size` settings. If the message count in
`/go2/bridge/topic_status` goes flat, or `ros2 bag record` stalls, that is the
first thing to suspect — switch to `publish_full_arrays:=false` and rely on
`/go2/lidar/points`.

Encoding the full array costs about 35 ms per cloud on the WebRTC callback
thread, roughly 17% of one core at 5 Hz. That is the price of float64 JSON; the
`PointCloud2` path is unaffected.

## Important limitation

`unitree_webrtc_connect` 2.1.2 automatically decodes only the `utlidar/*` binary stream using its built-in LiDAR decoder. Other binary topics, including uSLAM point clouds and grid-map data, are exposed by the library callback as raw bytes. This package deliberately keeps those as `UInt8MultiArray` rather than inventing a DDS/CDR schema. Once a sample rosbag is captured from the actual Go2 firmware, those converters can be added safely.
