#!/usr/bin/env python3
"""Depth-image obstacle safety arbiter for the BPX quadruped.

Sits between command sources (teleop, voice agent, autonomy) and
`bpx_bridge`:

    cmd_vel sources ──▶ [cmd_vel_unsafe] ──▶ bpx_safety ──▶ [cmd_vel] ──▶ bpx_bridge

Behavior:
    * Subscribes to a depth image (Insight 9 or any depth camera) and
      checks a forward zone for obstacles.
    * mode=stop : zero the command while an obstacle is in the zone.
    * mode=slow : scale the command down instead of stopping.
    * Hysteresis: the gate reopens only after the zone has been clear for
      `clear_time` seconds.
    * Publishes state on ~/active (Bool) and /diagnostics.

Zone geometry (camera optical frame: z forward, x right, y down):
    * Forward distance: 0 < z < stop_distance
    * Horizontal cone:  |atan2(x, z)| < cone_half_angle
    * Height band:      min_point_height < (-y) < max_point_height
      (-y = height of the point above the camera). The band excludes the
      floor and ceiling, which matters on a bouncing walking robot.
      TODO(sensor): calibrate min/max heights against the mounted camera
      height and gait bounce; the robot's own front legs may enter the
      zone — tune min_points / the band if they do.

Intrinsics come from CameraInfo when available (subscribe a few frames,
then unsubscribe). Until then, pinhole params are estimated from
declared image size / FOV parameters so the node works immediately.

No cv_bridge required: depth decoding uses numpy on raw data.
Supported encodings: 32FC1 (float, meters), 16UC1 (uint16, millimeters).
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


class BPXSafetyNode(Node):
    def __init__(self):
        super().__init__('bpx_safety')

        # ---------------- Parameters ----------------
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/infra1/camera_info')
        self.declare_parameter('cmd_vel_in', 'cmd_vel_unsafe')
        self.declare_parameter('cmd_vel_out', 'cmd_vel')
        self.declare_parameter('enabled', True)
        self.declare_parameter('mode', 'stop')          # 'stop' or 'slow'
        self.declare_parameter('stop_distance', 1.2)    # m, forward
        self.declare_parameter('cone_half_angle_deg', 30.0)
        self.declare_parameter('min_point_height', -0.30)  # m above camera
        self.declare_parameter('max_point_height', 1.50)   # m above camera
        self.declare_parameter('min_points', 8)         # points to trigger
        self.declare_parameter('pixel_step', 6)         # decimation factor
        self.declare_parameter('clear_time', 1.0)       # s of clear to release
        self.declare_parameter('slow_scale', 0.3)
        # Fallback intrinsics until CameraInfo arrives.
        self.declare_parameter('fallback_image_width', 1280)
        self.declare_parameter('fallback_image_height', 960)
        self.declare_parameter('fallback_hfov_deg', 115.6)   # Insight 9 depth FOV
        self.declare_parameter('fallback_vfov_deg', 96.0)    # TODO(sensor): confirm

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.cmd_vel_in = self.get_parameter('cmd_vel_in').value
        self.cmd_vel_out = self.get_parameter('cmd_vel_out').value
        self.enabled = bool(self.get_parameter('enabled').value)
        self.mode = self.get_parameter('mode').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.cone_half_angle = math.radians(
            float(self.get_parameter('cone_half_angle_deg').value))
        self.min_height = float(self.get_parameter('min_point_height').value)
        self.max_height = float(self.get_parameter('max_point_height').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.pixel_step = int(self.get_parameter('pixel_step').value)
        self.clear_time = float(self.get_parameter('clear_time').value)
        self.slow_scale = float(self.get_parameter('slow_scale').value)

        if self.mode not in ('stop', 'slow'):
            self.get_logger().warn(
                f"Unknown mode '{self.mode}', falling back to 'stop'.")
            self.mode = 'stop'

        # ---------------- Intrinsics ----------------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self._use_fallback_intrinsics()
        self._camera_info_seen = False

        # ---------------- Gate state ----------------
        self.blocked = False
        self.last_clear_time = time.monotonic()   # contiguous clear duration
        self.nearest_obstacle = float('inf')

        # ---------------- I/O ----------------
        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_out_pub = self.create_publisher(Twist, self.cmd_vel_out, 10)
        self.active_pub = self.create_publisher(Bool, '~/active', 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.cmd_in_sub = self.create_subscription(
            Twist, self.cmd_vel_in, self.on_cmd_vel, 10)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.on_depth, sensor_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info,
            sensor_qos)

        self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info(
            f'bpx_safety started: depth={self.depth_topic}, '
            f'{self.cmd_vel_in} -> {self.cmd_vel_out}, mode={self.mode}, '
            f'stop_distance={self.stop_distance:.2f} m')

    # ------------------------------------------------------------------
    # Intrinsics
    # ------------------------------------------------------------------
    def _use_fallback_intrinsics(self):
        w = float(self.get_parameter('fallback_image_width').value)
        h = float(self.get_parameter('fallback_image_height').value)
        hfov = math.radians(float(self.get_parameter('fallback_hfov_deg').value))
        vfov = math.radians(float(self.get_parameter('fallback_vfov_deg').value))
        self.fx = (w / 2.0) / math.tan(hfov / 2.0)
        self.fy = (h / 2.0) / math.tan(vfov / 2.0)
        self.cx = w / 2.0
        self.cy = h / 2.0

    def on_camera_info(self, msg: CameraInfo):
        if self._camera_info_seen:
            return
        k = msg.k
        if k[0] > 0 and k[4] > 0:
            self.fx, self.fy = k[0], k[4]
            self.cx, self.cy = k[2], k[5]
            self._camera_info_seen = True
            self.destroy_subscription(self.camera_info_sub)
            self.camera_info_sub = None
            self.get_logger().info(
                f'Got CameraInfo: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    # ------------------------------------------------------------------
    # Depth processing
    # ------------------------------------------------------------------
    def on_depth(self, msg: Image):
        if not self.enabled:
            return
        step = max(1, self.pixel_step)
        height, width = msg.height, msg.width
        raw = np.frombuffer(msg.data, dtype=self._numpy_dtype(msg.encoding))
        if raw.size < height * width:
            self.get_logger().warn('Depth image truncated; skipping frame.',
                                   throttle_duration_sec=5.0)
            return
        depth = raw[:height * width].reshape(height, width)[::step, ::step]

        if msg.encoding == '16UC1':
            depth = depth.astype(np.float32) / 1000.0
        elif msg.encoding == '32FC1':
            depth = depth.astype(np.float32)
        else:
            self.get_logger().warn(
                f'Unsupported depth encoding: {msg.encoding}',
                throttle_duration_sec=10.0)
            return

        valid = np.isfinite(depth) & (depth > 0.01) & (depth < self.stop_distance)
        if not valid.any():
            self._update_gate(False, float('inf'))
            return

        zs = depth[valid]
        # Grid pixel coordinates matching the decimated array.
        vs, us = np.meshgrid(
            np.arange(0, height, step), np.arange(0, width, step),
            indexing='ij')
        us = us[valid]
        vs = vs[valid]

        xs = (us - self.cx) * zs / self.fx
        ys = (vs - self.cy) * zs / self.fy

        horizontal_angle = np.arctan2(np.abs(xs), zs)
        height_above_camera = -ys

        in_zone = (
            (horizontal_angle < self.cone_half_angle)
            & (height_above_camera > self.min_height)
            & (height_above_camera < self.max_height)
        )
        n_points = int(in_zone.sum())
        nearest = float(zs[in_zone].min()) if n_points else float('inf')
        self._update_gate(n_points >= self.min_points, nearest)

    @staticmethod
    def _numpy_dtype(encoding):
        return np.uint16 if encoding == '16UC1' else np.float32

    # ------------------------------------------------------------------
    # Gate logic
    # ------------------------------------------------------------------
    def _update_gate(self, obstacle_present: bool, nearest: float):
        self.nearest_obstacle = nearest
        now = time.monotonic()
        if obstacle_present:
            self.last_clear_time = now
            if not self.blocked:
                self.blocked = True
                self.get_logger().warn(
                    f'Obstacle in stop zone at {nearest:.2f} m — '
                    f'{"stopping" if self.mode == "stop" else "slowing"} cmd_vel.',
                    throttle_duration_sec=1.0)
        elif self.blocked and (now - self.last_clear_time) >= self.clear_time:
            self.blocked = False
            self.get_logger().info(
                f'Stop zone clear for {self.clear_time:.1f}s — releasing cmd_vel.')
        self.active_pub.publish(Bool(data=self.blocked))

    def on_cmd_vel(self, msg: Twist):
        out = Twist()
        if not self.enabled or not self.blocked:
            out = msg
        elif self.mode == 'stop':
            self.get_logger().debug('cmd_vel zeroed by safety gate.',
                                    throttle_duration_sec=1.0)
            # out stays zero (linear+angular all 0)
        else:  # slow
            out = Twist()
            out.linear.x = msg.linear.x * self.slow_scale
            out.linear.y = msg.linear.y * self.slow_scale
            out.linear.z = msg.linear.z
            out.angular = msg.angular
            out.angular.z = msg.angular.z * self.slow_scale
        self.cmd_out_pub.publish(out)

    # ------------------------------------------------------------------
    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'bpx_safety'
        status.hardware_id = self.depth_topic
        if not self.enabled:
            status.level = DiagnosticStatus.STALE
            status.message = 'disabled'
        elif self.blocked:
            status.level = DiagnosticStatus.WARN
            status.message = f'obstacle stop active ({self.nearest_obstacle:.2f} m)'
        else:
            status.level = DiagnosticStatus.OK
            nearest = (f'{self.nearest_obstacle:.2f} m'
                       if math.isfinite(self.nearest_obstacle) else '> stop_distance')
            status.message = f'clear (nearest in range: {nearest})'
        status.values = [
            KeyValue(key='mode', value=self.mode),
            KeyValue(key='stop_distance_m', value=f'{self.stop_distance:.2f}'),
            KeyValue(key='intrinsics', value='camera_info' if self._camera_info_seen
                     else 'fallback'),
        ]
        array = DiagnosticArray()
        array.status.append(status)
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = BPXSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
