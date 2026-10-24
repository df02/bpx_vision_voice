#!/usr/bin/env python3
"""ROS 2 bridge node for the MirrorMe Black Panther X (BPX) quadruped.

Wraps the official `bpx_sdk_open` Python bindings
(https://github.com/mirrormerobotics/bpx_sdk_open).

Publishes:
    ~/joint_states   (sensor_msgs/JointState)   12 joints, SDK names/order
    ~/imu            (sensor_msgs/Imu)          body IMU from the robot
    ~/odom           (nav_msgs/Odometry)        leg odometry (SDK LegOdom)
    ~/motion_state   (std_msgs/String)          e.g. "Motion"
    ~/gait           (std_msgs/String)          e.g. "Walk"
    /diagnostics     (diagnostic_msgs/DiagnosticArray) battery + temperatures

Subscribes:
    cmd_vel          (geometry_msgs/Twist)      clamped, watchdog-guarded

Services (std_srvs/Trigger):
    stand_up, sit_down, damping, walk, run, pace, bound, pronk,
    bipedal, inv_bipedal, left_flip, right_flip,
    jump_up, jump_front, jump_back, jump_left, jump_right,
    zero_positions, enable_motion, reconnect

Safety model:
    * The SDK sends a complete zero command on every (re)connect and never
      replays old targets. The bridge mirrors this: after any observed
      disconnect/reconnect, motion is DISARMED and `cmd_vel` is ignored
      until the `enable_motion` service is called.
    * A cmd_vel watchdog publishes zero velocity if the command stream
      goes stale while motion is enabled.

If `bpx_sdk` is not importable, the node runs in MOCK mode (no robot
connection): state topics are populated with idle placeholders and all
commands are logged, so the workspace can be exercised on a desk.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

try:
    import bpx_sdk
    from bpx_sdk import (DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT,
                         DEFAULT_SERVER_IP)
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False

# Joint names in SDK order (bpx_sdk_open/include/motion_types.h JointIndex).
JOINT_NAMES = [
    'fl_hip_roll_joint', 'fl_hip_pitch_joint', 'fl_knee_joint',
    'fr_hip_roll_joint', 'fr_hip_pitch_joint', 'fr_knee_joint',
    'hl_hip_roll_joint', 'hl_hip_pitch_joint', 'hl_knee_joint',
    'hr_hip_roll_joint', 'hr_hip_pitch_joint', 'hr_knee_joint',
]
JOINT_COUNT = 12

# Idle standing-ish pose for mock mode (radians, placeholder values).
MOCK_JOINT_POSITIONS = [
    0.0, 0.8, -1.6,  # fl
    0.0, 0.8, -1.6,  # fr
    0.0, 0.8, -1.6,  # hl
    0.0, 0.8, -1.6,  # hr
]


class BPXBridgeNode(Node):
    def __init__(self):
        super().__init__('bpx_bridge')

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_ip', DEFAULT_SERVER_IP if SDK_AVAILABLE
                               else '10.21.20.1')
        self.declare_parameter('state_upload_rate', 100)       # 1..200 Hz
        self.declare_parameter('motion_command_rate', 50)      # Hz
        self.declare_parameter('joint_state_publish_rate', 100.0)
        self.declare_parameter('imu_publish_rate', 100.0)
        self.declare_parameter('odom_publish_rate', 50.0)
        self.declare_parameter('diagnostics_publish_rate', 1.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('cmd_vel_timeout', 0.5)         # s, 0 disables
        # Gait velocity limits [x, y, yaw] (bpx_sdk_open README, Walk gait).
        self.declare_parameter('max_vel_x', 1.5)
        self.declare_parameter('max_vel_y', 0.5)
        self.declare_parameter('max_vel_yaw', 2.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.state_upload_rate = int(self.get_parameter('state_upload_rate').value)
        self.motion_command_rate = int(self.get_parameter('motion_command_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.max_vel = (
            float(self.get_parameter('max_vel_x').value),
            float(self.get_parameter('max_vel_y').value),
            float(self.get_parameter('max_vel_yaw').value),
        )

        # ---------------- State ----------------
        self.lock = threading.Lock()
        self.robot = None               # bpx_sdk.MotionLevelControl or None
        self.was_connected = False
        self.motion_armed = False       # re-armed via enable_motion service
        self.velocity_enabled = False   # cmd_vel streaming active
        self.last_cmd_vel_time = 0.0
        self.zero_sent_by_watchdog = False
        self.mock_mode = not SDK_AVAILABLE

        # ---------------- Publishers ----------------
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.joint_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '~/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '~/odom', 10)
        self.motion_state_pub = self.create_publisher(String, '~/motion_state', latched)
        self.gait_pub = self.create_publisher(String, '~/gait', latched)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Subscriber ----------------
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # ---------------- Services ----------------
        simple_actions = {
            'stand_up': 'setStandUp',
            'sit_down': 'setSitDown',
            'damping': 'setDamping',
            'walk': 'setWalk',
            'run': 'setRunning',
            'pace': 'setPace',
            'bound': 'setBound',
            'pronk': 'setPronk',
            'bipedal': 'setBipedal',
            'inv_bipedal': 'setInvBipedal',
            'left_flip': 'setLeftFlip',
            'right_flip': 'setRightFlip',
            'jump_up': 'setUpJump',
            'jump_front': 'setFrontJump',
            'jump_back': 'setBackJump',
            'jump_left': 'setLeftJump',
            'jump_right': 'setRightJump',
        }
        for name, method in simple_actions.items():
            self.create_service(
                Trigger, f'~/{name}',
                self._make_trigger_service(method, name))
        self.create_service(Trigger, '~/zero_positions',
                            self.on_zero_positions)
        self.create_service(Trigger, '~/enable_motion', self.on_enable_motion)
        self.create_service(Trigger, '~/reconnect', self.on_reconnect)

        # ---------------- SDK client ----------------
        if self.mock_mode:
            self.get_logger().warn(
                'bpx_sdk not importable - running in MOCK mode. '
                'State topics will contain placeholder data and all '
                'commands are logged only.')
        else:
            self._create_client()

        # ---------------- Timers ----------------
        self._add_rate_timer('joint_state_publish_rate', self.publish_joint_states)
        self._add_rate_timer('imu_publish_rate', self.publish_imu)
        self._add_rate_timer('odom_publish_rate', self.publish_odom)
        self._add_rate_timer('diagnostics_publish_rate', self.publish_diagnostics)
        self.create_timer(1.0, self.check_connection)          # watchdog conn
        if self.cmd_vel_timeout > 0.0:
            self.create_timer(min(0.1, self.cmd_vel_timeout / 2.0),
                              self.cmd_vel_watchdog)

        self.get_logger().info(
            f'bpx_bridge started (ip={self.robot_ip}, '
            f'mock={self.mock_mode})')

    # ------------------------------------------------------------------
    # SDK connection management
    # ------------------------------------------------------------------
    def _create_client(self):
        """Create (or re-create) the MotionLevelControl client."""
        try:
            robot = bpx_sdk.MotionLevelControl()
            robot.setRobotIp(self.robot_ip)
            robot.setRobotStateUploadPort(DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT)
            robot.setTcpLocalPort(0)
            robot.setRobotStateUploadRate(self.state_upload_rate)
            robot.setMotionCommandRate(self.motion_command_rate)
            with self.lock:
                ok = robot.connect()
                self.robot = robot
            if ok:
                self.get_logger().info(f'Connected to BPX at {self.robot_ip}')
                self._log_identity()
            else:
                self.get_logger().warn(
                    f'Initial connect to {self.robot_ip} failed; the SDK '
                    'will keep retrying in the background.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to create SDK client: {exc}')
            self.robot = None

    def check_connection(self):
        """1 Hz supervision: track disconnects, re-arm logic, retry create."""
        if self.mock_mode:
            return
        with self.lock:
            robot = self.robot
            connected = bool(robot.isConnected()) if robot is not None else False

        if connected and not self.was_connected:
            self.get_logger().info('BPX link is live.')
        elif not connected and self.was_connected:
            # The SDK zeroed commands on reconnect. Require explicit re-arm.
            self.get_logger().warn(
                'BPX link lost. Motion disarmed; call the '
                'enable_motion service to re-arm after reconnection.')
            self.motion_armed = False
            self.velocity_enabled = False
        elif not connected and not self.was_connected and self.robot is None:
            self._create_client()
        self.was_connected = connected

    def _log_identity(self):
        try:
            sn = self.robot.getRobotSerialNumber()
            model = self.robot.getRobotModel()
            if sn is not None and model is not None:
                model_name = bpx_sdk.RobotModel(model).name
                self.get_logger().info(f'Robot identity: SN={sn}, model={model_name}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'Identity query failed: {exc}')

    # ------------------------------------------------------------------
    # cmd_vel + watchdog
    # ------------------------------------------------------------------
    def on_cmd_vel(self, msg: Twist):
        if self.motion_armed and not self.velocity_enabled:
            self.get_logger().warn('cmd_vel ignored: motion is disarmed. '
                                   'Call ~/enable_motion first.')
            return
        x = max(-self.max_vel[0], min(self.max_vel[0], msg.linear.x))
        y = max(-self.max_vel[1], min(self.max_vel[1], msg.linear.y))
        yaw = max(-self.max_vel[2], min(self.max_vel[2], msg.angular.z))

        if self.mock_mode:
            self.get_logger().info(f'[mock] setVelocity({x:.2f}, {y:.2f}, {yaw:.2f})')
            return

        try:
            with self.lock:
                self.robot.setVelocity(float(x), float(y), float(yaw))
            self.last_cmd_vel_time = time.monotonic()
            self.velocity_enabled = True
            self.zero_sent_by_watchdog = False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'setVelocity failed: {exc}')

    def cmd_vel_watchdog(self):
        """Zero velocity when the cmd_vel stream goes stale."""
        if self.mock_mode or not self.velocity_enabled:
            return
        if time.monotonic() - self.last_cmd_vel_time < self.cmd_vel_timeout:
            return
        if self.zero_sent_by_watchdog:
            return
        try:
            with self.lock:
                self.robot.setVelocity(0.0, 0.0, 0.0)
            self.zero_sent_by_watchdog = True
            self.get_logger().warn(
                f'cmd_vel stale > {self.cmd_vel_timeout}s: sent zero velocity.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Watchdog zero-velocity failed: {exc}')

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _make_trigger_service(self, sdk_method, human_name):
        def callback(request, response):
            if self.mock_mode:
                self.get_logger().info(f'[mock] {human_name}')
                response.success = True
                response.message = 'mock mode'
                return response
            if not self.motion_armed:
                response.success = False
                response.message = 'motion is disarmed; call enable_motion first'
                return response
            try:
                with self.lock:
                    getattr(self.robot, sdk_method)()
                response.success = True
                response.message = f'{human_name} sent'
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'{human_name} failed: {exc}')
                response.success = False
                response.message = str(exc)
            return response
        return callback

    def on_zero_positions(self, request, response):
        """Zero-position flag. Feet/shanks/joints must be on the ground!"""
        if self.mock_mode:
            self.get_logger().info('[mock] zero_positions')
            response.success, response.message = True, 'mock mode'
            return response
        try:
            with self.lock:
                self.robot.setZeroPositionsFlag()
            response.success, response.message = True, 'zeroing command sent'
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'zero_positions failed: {exc}')
            response.success, response.message = False, str(exc)
        return response

    def on_enable_motion(self, request, response):
        """Re-arm motion after connect/reconnect, and (re)enable velocity ctrl."""
        if self.mock_mode:
            self.motion_armed = True
            response.success, response.message = True, 'mock mode: armed'
            return response
        with self.lock:
            connected = self.robot is not None and self.robot.isConnected()
            if connected:
                try:
                    self.robot.setVelocityControlFlag(True)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f'setVelocityControlFlag failed: {exc}')
        if not connected:
            response.success = False
            response.message = 'not connected to robot'
            return response
        self.motion_armed = True
        self.get_logger().info('Motion ARMED. cmd_vel and motion services active.')
        response.success = True
        response.message = 'motion armed'
        return response

    def on_reconnect(self, request, response):
        if self.mock_mode:
            response.success, response.message = True, 'mock mode'
            return response
        try:
            with self.lock:
                if self.robot is not None:
                    self.robot.disconnect()
            self._create_client()
            self.motion_armed = False
            response.success = True
            response.message = 'reconnect requested; motion disarmed'
        except Exception as exc:  # noqa: BLE001
            response.success, response.message = False, str(exc)
        return response

    # ------------------------------------------------------------------
    # State publishing
    # ------------------------------------------------------------------
    def _add_rate_timer(self, param_name, callback):
        rate = float(self.get_parameter(param_name).value)
        if rate > 0.0:
            self.create_timer(1.0 / rate, callback)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        if self.mock_mode:
            msg.position = list(MOCK_JOINT_POSITIONS)
            msg.velocity = [0.0] * JOINT_COUNT
            msg.effort = [0.0] * JOINT_COUNT
        else:
            pos = self.robot.getJointPosition() if self.robot else None
            vel = self.robot.getJointVelocity() if self.robot else None
            eff = self.robot.getJointTorque() if self.robot else None
            msg.position = list(pos) if pos else [float('nan')] * JOINT_COUNT
            msg.velocity = list(vel) if vel else []
            msg.effort = list(eff) if eff else []
        self.joint_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        if self.mock_mode:
            msg.orientation.w = 1.0
        else:
            quat = self.robot.getImuQuat() if self.robot else None
            acc = self.robot.getImuAcc() if self.robot else None
            omega = self.robot.getImuOmega() if self.robot else None
            # SDK quaternion order is x/y/z/w (motion_types.h LegOdom comment).
            if quat and len(quat) == 4:
                msg.orientation.x, msg.orientation.y = quat[0], quat[1]
                msg.orientation.z, msg.orientation.w = quat[2], quat[3]
            if acc:
                msg.linear_acceleration.x, msg.linear_acceleration.y = acc[0], acc[1]
                msg.linear_acceleration.z = acc[2]
            if omega:
                msg.angular_velocity.x, msg.angular_velocity.y = omega[0], omega[1]
                msg.angular_velocity.z = omega[2]
        # TODO(sensor): covariance from datasheet once known.
        self.imu_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        if self.mock_mode:
            msg.pose.pose.orientation.w = 1.0
        else:
            odom = self.robot.getLegOdom() if self.robot else None
            if odom:
                pos = odom.get('position')
                ori = odom.get('orientation')
                vel = odom.get('velocity_body')
                ang = odom.get('angular_velocity')
                if pos:
                    msg.pose.pose.position.x = pos[0]
                    msg.pose.pose.position.y = pos[1]
                    msg.pose.pose.position.z = pos[2]
                if ori and len(ori) == 4:
                    msg.pose.pose.orientation.x = ori[0]
                    msg.pose.pose.orientation.y = ori[1]
                    msg.pose.pose.orientation.z = ori[2]
                    msg.pose.pose.orientation.w = ori[3]
                if vel:
                    msg.twist.twist.linear.x = vel[0]
                    msg.twist.twist.linear.y = vel[1]
                if ang:
                    msg.twist.twist.angular.z = ang[2]
        # TODO(sensor): replace placeholder covariances with measured values.
        msg.pose.covariance[0] = 0.01   # x
        msg.pose.covariance[7] = 0.01   # y
        msg.pose.covariance[35] = 0.05  # yaw
        msg.twist.covariance[0] = 0.01  # vx
        msg.twist.covariance[7] = 0.01  # vy
        msg.twist.covariance[35] = 0.05  # vyaw
        self.odom_pub.publish(msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def publish_diagnostics(self):
        array = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'bpx_robot'
        status.hardware_id = self.robot_ip

        if self.mock_mode:
            status.level = DiagnosticStatus.STALE
            status.message = 'mock mode'
            status.values = [KeyValue(key='mode', value='mock')]
        else:
            connected = self.robot is not None and self.robot.isConnected()
            battery = self.robot.getBatteryLevel() if self.robot else None
            current = self.robot.getBatteryCurrent() if self.robot else None
            motor_temps = self.robot.getMotorTemperature() if self.robot else None
            status.level = (DiagnosticStatus.OK if connected
                            else DiagnosticStatus.ERROR)
            status.message = 'connected' if connected else 'disconnected'
            if battery is not None:
                status.values.append(
                    KeyValue(key='battery_percent', value=str(battery)))
            if current is not None:
                status.values.append(
                    KeyValue(key='battery_current_a', value=str(current)))
            if motor_temps:
                hottest = max(motor_temps)
                status.values.append(
                    KeyValue(key='max_motor_temp_c', value=str(hottest)))
                if hottest > 70.0:
                    status.level = DiagnosticStatus.WARN
            state = self.robot.getCurrentMotionState() if self.robot else None
            gait = self.robot.getCurrentGait() if self.robot else None
            if state is not None:
                status.values.append(KeyValue(
                    key='motion_state', value=self._enum_name(bpx_sdk.MotionState, state)))
            if gait is not None:
                status.values.append(KeyValue(
                    key='gait', value=self._enum_name(bpx_sdk.MotionGait, gait)))
            status.values.append(KeyValue(
                key='motion_armed', value=str(self.motion_armed)))

        array.status.append(status)
        self.diag_pub.publish(array)

        # Latched string topics for convenience / GUIs.
        state_msg = String()
        gait_msg = String()
        if self.mock_mode:
            state_msg.data, gait_msg.data = 'Mock', 'Unknown'
        else:
            state = self.robot.getCurrentMotionState() if self.robot else None
            gait = self.robot.getCurrentGait() if self.robot else None
            state_msg.data = self._enum_name(bpx_sdk.MotionState, state) or 'Unknown'
            gait_msg.data = self._enum_name(bpx_sdk.MotionGait, gait) or 'Unknown'
        self.motion_state_pub.publish(state_msg)
        self.gait_pub.publish(gait_msg)

    @staticmethod
    def _enum_name(enum_type, raw_value):
        if raw_value is None:
            return None
        try:
            return enum_type(raw_value).name
        except ValueError:
            return f'Unknown({raw_value})'

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.robot is not None:
            try:
                with self.lock:
                    self.robot.disconnect()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BPXBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
