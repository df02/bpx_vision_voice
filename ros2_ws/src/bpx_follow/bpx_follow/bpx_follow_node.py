#!/usr/bin/env python3
"""Person-following node for the BPX quadruped (Insight 9 camera).

Concept of operation:
    * The Insight 9 runs person detection ON-DEVICE (YOLO flashed from the
      looper.local dashboard) and publishes detections.
    * This node picks the best person detection, computes its bearing
      (horizontal angle) from the RGB bbox + RGB intrinsics, and its
      distance by sampling the depth image along that bearing.
    * A P-controller publishes body-frame velocity commands to
      `cmd_vel_unsafe` — i.e. THROUGH the bpx_safety obstacle gate, which
      can still stop or slow everything.

RGB/depth alignment:
    RGB (1088x1920 MJPEG portrait) and depth (544x640) have different
    intrinsics and the camera publishes no aligned depth-to-color image.
    Instead of pixel mapping, the bbox center is converted to a bearing /
    elevation ANGLE via RGB intrinsics, then re-projected into the depth
    image with the depth intrinsics (angle is frame-independent, and both
    sensors share the camera's forward axis closely enough for following).

Safety behavior:
    * Starts DISABLED — nothing is published until ~/enable is called.
    * Person lost > lost_timeout  -> zero command, status "LOST".
    * Depth stale > depth_timeout -> zero command (can't know distance).
    * Intrinsics not yet received -> zero command.
    * Distance < stop_distance    -> zero linear (hold position).
    * The obstacle gate (bpx_safety) remains in the loop regardless.

TODO(sensor): confirm the v2.1.4 detection topic name and message type.
This node assumes vision_msgs/Detection2DArray; adjust `detection_topic`
and `detection_type` if the camera publishes something else.
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from vision_msgs.msg import Detection2DArray
    HAVE_VISION_MSGS = True
except ImportError:
    HAVE_VISION_MSGS = False


class BPXFollowNode(Node):
    def __init__(self):
        super().__init__('bpx_follow')

        # ---------------- Parameters ----------------
        self.declare_parameter('detection_topic', '/camera/camera/detections')
        self.declare_parameter('detection_type', 'vision_msgs/msg/Detection2DArray')
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('rgb_camera_info_topic',
                               '/camera/camera/color/camera_info')
        self.declare_parameter('depth_camera_info_topic',
                               '/camera/camera/infra1/camera_info')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_unsafe')
        self.declare_parameter('status_topic', '~/status')

        self.declare_parameter('target_class', 'person')
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('enabled_on_start', False)

        # Following behavior
        self.declare_parameter('follow_distance', 1.5)   # m, desired gap
        self.declare_parameter('stop_distance', 1.0)     # m, too close: hold
        self.declare_parameter('max_speed', 1.0)         # m/s (Walk limit 1.5)
        self.declare_parameter('max_yaw', 1.0)           # rad/s
        self.declare_parameter('k_vel', 0.8)             # 1/s velocity gain
        self.declare_parameter('k_yaw', 1.5)             # 1/s yaw gain

        # Robustness
        self.declare_parameter('lost_timeout', 1.0)      # s without detection
        self.declare_parameter('depth_timeout', 0.5)     # s without depth
        self.declare_parameter('min_depth_pixels', 15)   # valid px in sample box
        self.declare_parameter('depth_sample_radius', 25)  # px half-size of box
        self.declare_parameter('control_rate', 20.0)     # Hz

        gp = self.get_parameter
        self.detection_topic = gp('detection_topic').value
        self.detection_type = gp('detection_type').value
        self.depth_topic = gp('depth_topic').value
        self.rgb_ci_topic = gp('rgb_camera_info_topic').value
        self.depth_ci_topic = gp('depth_camera_info_topic').value
        self.cmd_vel_topic = gp('cmd_vel_topic').value
        self.status_topic = gp('status_topic').value
        self.target_class = gp('target_class').value
        self.min_score = float(gp('min_score').value)
        self.follow_distance = float(gp('follow_distance').value)
        self.stop_distance = float(gp('stop_distance').value)
        self.max_speed = float(gp('max_speed').value)
        self.max_yaw = float(gp('max_yaw').value)
        self.k_vel = float(gp('k_vel').value)
        self.k_yaw = float(gp('k_yaw').value)
        self.lost_timeout = float(gp('lost_timeout').value)
        self.depth_timeout = float(gp('depth_timeout').value)
        self.min_depth_pixels = int(gp('min_depth_pixels').value)
        self.sample_radius = int(gp('depth_sample_radius').value)

        # ---------------- State ----------------
        self.enabled = bool(gp('enabled_on_start').value)
        self.fx_rgb = self.fy_rgb = self.cx_rgb = self.cy_rgb = None
        self.fx_d = self.fy_d = self.cx_d = self.cy_d = None
        self.depth_img = None            # np.float32 meters, or None
        self.depth_stamp = 0.0
        self.last_detection = None       # (stamp, u, v, score)
        self.last_zero_sent = True

        # ---------------- I/O ----------------
        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        if HAVE_VISION_MSGS and self.detection_type == 'vision_msgs/msg/Detection2DArray':
            self.det_sub = self.create_subscription(
                Detection2DArray, self.detection_topic, self.on_detections,
                sensor_qos)
        else:
            self.det_sub = None
            self.get_logger().error(
                f"Detection type '{self.detection_type}' not supported by this "
                'build (only vision_msgs/Detection2DArray) or vision_msgs is '
                'missing. Node will stay idle.')

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.on_depth, sensor_qos)
        self.rgb_ci_sub = self.create_subscription(
            CameraInfo, self.rgb_ci_topic, self.on_rgb_camera_info, sensor_qos)
        self.depth_ci_sub = self.create_subscription(
            CameraInfo, self.depth_ci_topic, self.on_depth_camera_info, sensor_qos)
        self.enable_srv = self.create_service(
            SetBool, '~/enable', self.on_enable)

        self.create_timer(1.0 / float(gp('control_rate').value), self.control_loop)
        self._publish_status('IDLE (disabled)' if not self.enabled else 'ACTIVE')

        self.get_logger().info(
            f'bpx_follow started: detections={self.detection_topic}, '
            f'depth={self.depth_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'enabled={self.enabled}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def on_rgb_camera_info(self, msg: CameraInfo):
        if self.fx_rgb is None and msg.k[0] > 0:
            self.fx_rgb, self.fy_rgb = msg.k[0], msg.k[4]
            self.cx_rgb, self.cy_rgb = msg.k[2], msg.k[5]
            self.get_logger().info(f'RGB intrinsics: fx={self.fx_rgb:.1f}')

    def on_depth_camera_info(self, msg: CameraInfo):
        if self.fx_d is None and msg.k[0] > 0:
            self.fx_d, self.fy_d = msg.k[0], msg.k[4]
            self.cx_d, self.cy_d = msg.k[2], msg.k[5]
            self.get_logger().info(f'Depth intrinsics: fx={self.fx_d:.1f}')

    def on_depth(self, msg: Image):
        raw = np.frombuffer(msg.data,
                            dtype=np.uint16 if msg.encoding == '16UC1'
                            else np.float32)
        h, w = msg.height, msg.width
        if raw.size < h * w:
            return
        depth = raw[:h * w].reshape(h, w)
        if msg.encoding == '16UC1':
            depth = depth.astype(np.float32) / 1000.0
        else:
            depth = depth.astype(np.float32)
        self.depth_img = depth
        self.depth_stamp = time.monotonic()

    def on_detections(self, msg: Detection2DArray):
        best = None  # (score, area, u, v)
        for det in msg.detections:
            hyp = self._best_hypothesis(det)
            if hyp is None or hyp['score'] < self.min_score:
                continue
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)
            area = float(det.bbox.size_x * det.bbox.size_y)
            # Prefer the most confident, then the largest (closest) target.
            key = (hyp['score'], area)
            if best is None or key > (best[0], best[1]):
                best = (hyp['score'], area, u, v)
        if best is not None:
            self.last_detection = (time.monotonic(), best[2], best[3], best[0])

    @staticmethod
    def _best_hypothesis(det):
        """Handle both Humble (hypothesis.class_id/score) and legacy (id/score)."""
        if not det.results:
            return None
        r = det.results[0]
        hyp = getattr(r, 'hypothesis', r)
        class_id = getattr(hyp, 'class_id', getattr(hyp, 'id', ''))
        score = float(getattr(hyp, 'score', 0.0))
        if class_id and class_id != 'person' and class_id != '0':
            return None
        return {'class_id': class_id, 'score': score}

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self._send_cmd(0.0, 0.0, force=True)
        self._publish_status('ACTIVE' if self.enabled else 'IDLE (disabled)')
        self.get_logger().info(f'Following {"ENABLED" if self.enabled else "DISABLED"}')
        response.success = True
        response.message = 'enabled' if self.enabled else 'disabled'
        return response

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        now = time.monotonic()

        if not self.enabled:
            return

        if self.fx_rgb is None or self.fx_d is None:
            self._publish_status('WAITING: no camera_info yet')
            self._send_cmd(0.0, 0.0)
            return

        if (self.last_detection is None
                or now - self.last_detection[0] > self.lost_timeout):
            self._publish_status('LOST: no person detected')
            self._send_cmd(0.0, 0.0)
            return

        if self.depth_img is None or now - self.depth_stamp > self.depth_timeout:
            self._publish_status('WAITING: depth stale')
            self._send_cmd(0.0, 0.0)
            return

        _, u_rgb, v_rgb, score = self.last_detection

        # --- RGB bbox center -> bearing/elevation angles ---
        bearing = math.atan2(u_rgb - self.cx_rgb, self.fx_rgb)   # +right
        elevation = math.atan2(v_rgb - self.cy_rgb, self.fy_rgb)  # +down

        # --- Angles -> depth image pixel (angle is frame-independent) ---
        u_d = int(round(self.cx_d + math.tan(bearing) * self.fx_d))
        v_d = int(round(self.cy_d + math.tan(elevation) * self.fy_d))
        h, w = self.depth_img.shape
        u_d = max(0, min(w - 1, u_d))
        v_d = max(0, min(h - 1, v_d))
        r = self.sample_radius
        box = self.depth_img[max(0, v_d - r):v_d + r, max(0, u_d - r):u_d + r]
        valid = box[np.isfinite(box) & (box > 0.05)]
        if valid.size < self.min_depth_pixels:
            self._publish_status('WAITING: no depth on target')
            self._send_cmd(0.0, 0.0)
            return
        distance = float(np.median(valid))

        # --- Velocity commands (body frame) ---
        # Person to the right (bearing > 0) -> turn right (negative yaw).
        yaw = max(-self.max_yaw, min(self.max_yaw, -self.k_yaw * bearing))
        if distance < self.stop_distance:
            vel = 0.0
            state = f'HOLDING {distance:.2f} m (too close)'
        else:
            err = distance - self.follow_distance
            vel = max(0.0, min(self.max_speed, self.k_vel * err))
            state = (f'FOLLOWING {distance:.2f} m '
                     f'bearing {math.degrees(bearing):+.0f} deg '
                     f'vel {vel:.2f} yaw {yaw:+.2f}')

        self._send_cmd(vel, yaw)
        self._publish_status(state)

    # ------------------------------------------------------------------
    def _send_cmd(self, vel: float, yaw: float, force: bool = False):
        """Publish cmd_vel; suppress duplicate zero commands unless forced."""
        if vel == 0.0 and yaw == 0.0 and self.last_zero_sent and not force:
            return
        msg = Twist()
        msg.linear.x = float(vel)
        msg.angular.z = float(yaw)
        self.cmd_pub.publish(msg)
        self.last_zero_sent = (vel == 0.0 and yaw == 0.0)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BPXFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
