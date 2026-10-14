"""Detect a person in RGB-D data and update a Nav2 following goal."""

import math

from care_robot.geometry import compute_follow_goal
from care_robot.geometry import image_message_to_array
from care_robot.geometry import project_pixel_to_optical_frame
from care_robot.geometry import sample_depth
import cv2
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs
import tf2_ros
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException

try:
    import mediapipe as mp
except ImportError:  # Kept optional so ROS tooling can still inspect the package.
    mp = None


class HumanFollower(Node):
    """Turn RGB-D human detections into navigation goals."""

    def __init__(self) -> None:
        """Initialize parameters, ROS interfaces, TF, and MediaPipe."""
        super().__init__('human_follower')

        self._declare_parameters()
        self._read_parameters()

        self._depth_image = None
        self._camera_intrinsics = None
        self._target_lost = False
        self._has_seen_target = False
        self._last_rgb_time = None
        self._search_direction = 1.0
        self._pose_detector = None
        self._hog_detector = None
        self._initialize_detector()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self)

        self._goal_publisher = self.create_publisher(
            PoseStamped, self._goal_topic, 10)
        self._cmd_vel_publisher = self.create_publisher(
            Twist, self._cmd_vel_topic, 10)
        self._navigate_client = ActionClient(
            self, NavigateToPose, self._navigate_action)

        self._rgb_subscription = self.create_subscription(
            Image,
            self._rgb_topic,
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            self._depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        now = self.get_clock().now()
        self._last_goal_time = now
        self._last_detection_time = now
        self._active_goal_handle = None
        self._goal_request_pending = False
        self._goal_epoch = 0
        self._canceling_goals = []
        self._lost_timer = self.create_timer(0.1, self._check_target_timeout)

        self.get_logger().info('Human follower node is ready')

    def _declare_parameters(self) -> None:
        """Declare all public ROS parameters in one place."""
        self.declare_parameter('rgb_topic', 'robot_camera_sensor/image')
        self.declare_parameter(
            'depth_topic', 'robot_camera_sensor/depth_image')
        self.declare_parameter(
            'camera_info_topic', 'robot_camera_sensor/camera_info')
        self.declare_parameter('goal_topic', 'follow_goal')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel_nav')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('follow_distance', 2.2)
        self.declare_parameter('goal_update_period', 1.0)
        self.declare_parameter('detection_timeout', 3.0)
        self.declare_parameter('transform_timeout', 0.2)
        self.declare_parameter('search_when_lost', True)
        self.declare_parameter('search_angular_speed', 0.4)
        self.declare_parameter('search_linear_speed', 0.1)
        self.declare_parameter('min_turning_radius', 0.65)
        self.declare_parameter('search_duration', 12.0)
        self.declare_parameter('publish_goal_topic', True)
        self.declare_parameter('send_nav2_goal', True)
        self.declare_parameter('show_debug_view', False)
        self.declare_parameter('detector_backend', 'auto')
        self.declare_parameter('depth_window_radius', 2)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)

    def _read_parameters(self) -> None:
        """Cache parameters used in high-frequency callbacks."""
        def value(name):
            return self.get_parameter(name).value

        self._rgb_topic = value('rgb_topic')
        self._depth_topic = value('depth_topic')
        self._camera_info_topic = value('camera_info_topic')
        self._goal_topic = value('goal_topic')
        self._cmd_vel_topic = value('cmd_vel_topic')
        self._navigate_action = value('navigate_action')
        self._camera_frame = value('camera_frame')
        self._global_frame = value('global_frame')
        self._robot_base_frame = value('robot_base_frame')
        self._follow_distance = float(value('follow_distance'))
        self._goal_update_period = float(value('goal_update_period'))
        self._detection_timeout = float(value('detection_timeout'))
        self._transform_timeout = Duration(
            seconds=float(value('transform_timeout')))
        self._search_when_lost = bool(value('search_when_lost'))
        self._search_angular_speed = float(value('search_angular_speed'))
        self._search_linear_speed = float(value('search_linear_speed'))
        self._min_turning_radius = float(value('min_turning_radius'))
        self._search_duration = float(value('search_duration'))
        if (self._min_turning_radius <= 0.0
                or self._search_linear_speed < 0.0
                or self._search_duration < 0.0):
            raise ValueError('Search requires positive radius and nonnegative speed/duration')
        self._publish_goal_topic = bool(value('publish_goal_topic'))
        self._send_nav2_goal = bool(value('send_nav2_goal'))
        self._show_debug_view = bool(value('show_debug_view'))
        self._detector_backend = str(value('detector_backend')).lower()
        self._depth_window_radius = int(value('depth_window_radius'))
        self._depth_scale = float(value('depth_scale'))
        self._min_detection_confidence = float(
            value('min_detection_confidence'))
        self._min_tracking_confidence = float(
            value('min_tracking_confidence'))

    def _initialize_detector(self) -> None:
        """Select MediaPipe when available and otherwise use OpenCV HOG."""
        backend = self._detector_backend
        if backend == 'auto':
            backend = 'mediapipe' if mp is not None else 'hog'

        if backend == 'mediapipe':
            if mp is None:
                raise RuntimeError(
                    'detector_backend=mediapipe requires the optional '
                    'MediaPipe Python package')
            self._pose_detector = mp.solutions.pose.Pose(
                min_detection_confidence=self._min_detection_confidence,
                min_tracking_confidence=self._min_tracking_confidence,
            )
            self.get_logger().info('Using MediaPipe pose detection')
            return

        if backend == 'hog':
            self._hog_detector = cv2.HOGDescriptor()
            self._hog_detector.setSVMDetector(
                cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().info(
                'Using OpenCV HOG person detection (MediaPipe is optional)')
            return

        raise ValueError(
            'detector_backend must be one of: auto, mediapipe, hog')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        """Store the latest calibrated pinhole camera intrinsics."""
        intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])
        if intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0:
            self.get_logger().warning('Ignoring invalid camera intrinsics')
            return
        if self._camera_intrinsics is None:
            self.get_logger().info(
                'Camera intrinsics received: '
                f'fx={intrinsics[0]:.1f}, fy={intrinsics[1]:.1f}')
        self._camera_intrinsics = intrinsics

    def _depth_callback(self, message: Image) -> None:
        """Convert and cache the latest registered depth image in metres."""
        try:
            depth_image = image_message_to_array(message)
        except ValueError as error:
            self.get_logger().error(f'Depth conversion failed: {error}')
            return

        depth_image = np.asarray(depth_image)
        if np.issubdtype(depth_image.dtype, np.integer):
            depth_image = depth_image.astype(np.float32) * self._depth_scale
        self._depth_image = depth_image

    def _rgb_callback(self, message: Image) -> None:
        """Run pose detection and publish a following goal."""
        self._last_rgb_time = self.get_clock().now()
        if self._camera_intrinsics is None or self._depth_image is None:
            return

        try:
            frame = self._rgb_to_bgr(message)
        except ValueError as error:
            self.get_logger().error(f'RGB conversion failed: {error}')
            return

        person_center = self._detect_person_center(frame)
        if person_center is None:
            self._handle_missing_detection()
            self._show_debug_frame(frame)
            return
        pixel_x, pixel_y = person_center
        self._remember_search_direction(pixel_x, frame.shape[1])

        depth = sample_depth(
            self._depth_image,
            pixel_x,
            pixel_y,
            self._depth_window_radius,
        )
        if depth is None:
            self._handle_missing_detection()
            self._show_debug_frame(frame)
            return

        human_point = self._point_in_global_frame(
            message, pixel_x, pixel_y, depth)
        if human_point is None:
            self._handle_missing_detection()
            self._show_debug_frame(frame)
            return

        robot_position = self._robot_position_in_global_frame()
        if robot_position is None:
            self._show_debug_frame(frame)
            return

        now = self.get_clock().now()
        self._last_detection_time = now
        self._has_seen_target = True
        reacquired = self._target_lost
        self._stop_search()

        goal = compute_follow_goal(
            robot_position,
            (human_point.point.x, human_point.point.y),
            self._follow_distance,
        )
        elapsed = (now - self._last_goal_time).nanoseconds / 1e9
        if goal is not None and (
                reacquired or elapsed >= self._goal_update_period):
            goal_message = self._create_goal_message(goal, now)
            self._dispatch_goal(goal_message)
            self._last_goal_time = now
            self.get_logger().info(
                f'Human=({human_point.point.x:.2f}, '
                f'{human_point.point.y:.2f}), '
                f'goal=({goal[0]:.2f}, {goal[1]:.2f})')

        if self._show_debug_view:
            cv2.circle(frame, (pixel_x, pixel_y), 8, (0, 0, 255), -1)
        self._show_debug_frame(frame)

    @staticmethod
    def _rgb_to_bgr(message):
        """Decode a ROS image and normalize it to OpenCV BGR ordering."""
        frame = image_message_to_array(message)
        encoding = message.encoding.lower()
        if encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        if encoding in {'bgr8', '8uc3'}:
            return frame
        raise ValueError(
            f'RGB stream uses unsupported encoding {message.encoding!r}')

    def _detect_person_center(self, frame):
        """Return an upper-body image point from the configured detector."""
        if self._pose_detector is not None:
            results = self._pose_detector.process(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if results.pose_landmarks is None:
                return None
            landmarks = results.pose_landmarks.landmark
            left = landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
            right = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
            return (
                int((left.x + right.x) * frame.shape[1] / 2.0),
                int((left.y + right.y) * frame.shape[0] / 2.0),
            )

        boxes, _ = self._hog_detector.detectMultiScale(
            frame,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        if len(boxes) == 0:
            return None
        x, y, width, height = max(
            boxes, key=lambda box: int(box[2]) * int(box[3]))
        return int(x + width / 2), int(y + height * 0.35)

    def _point_in_global_frame(
            self, message: Image, pixel_x: int, pixel_y: int,
            depth: float):
        """Project an image pixel and transform it into the global frame."""
        point_xyz = project_pixel_to_optical_frame(
            pixel_x, pixel_y, depth, *self._camera_intrinsics)
        point = PointStamped()
        point.header.frame_id = self._camera_frame or message.header.frame_id
        point.header.stamp = message.header.stamp
        point.point.x, point.point.y, point.point.z = point_xyz

        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                point.header.frame_id,
                # The callback and TF listener share one executor. Requesting
                # the latest transform avoids waiting for TF while the
                # callback itself prevents the listener from advancing.
                Time(),
                timeout=self._transform_timeout,
            )
            return tf2_geometry_msgs.do_transform_point(point, transform)
        except (LookupException, ConnectivityException,
                ExtrapolationException) as error:
            self.get_logger().warning(f'Camera TF unavailable: {error}')
            return None

    def _robot_position_in_global_frame(self):
        """Return the robot position in the configured global frame."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                Time(),
                timeout=self._transform_timeout,
            )
        except (LookupException, ConnectivityException,
                ExtrapolationException) as error:
            self.get_logger().warning(f'Robot TF unavailable: {error}')
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def _create_goal_message(self, goal, stamp) -> PoseStamped:
        """Build a map-frame pose facing from the robot toward the person."""
        goal_x, goal_y, yaw = goal
        message = PoseStamped()
        message.header.frame_id = self._global_frame
        message.header.stamp = stamp.to_msg()
        message.pose.position.x = goal_x
        message.pose.position.y = goal_y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def _dispatch_goal(self, goal: PoseStamped) -> None:
        """Publish the goal for observability and send it to Nav2."""
        if self._publish_goal_topic:
            self._goal_publisher.publish(goal)
        if (not self._send_nav2_goal or self._goal_request_pending
                or self._canceling_goals or self._target_lost):
            return
        if not self._navigate_client.server_is_ready():
            self.get_logger().warning(
                f'Nav2 action {self._navigate_action!r} is not ready')
            return

        request = NavigateToPose.Goal()
        request.pose = goal
        self._goal_request_pending = True
        epoch = self._goal_epoch
        future = self._navigate_client.send_goal_async(request)
        future.add_done_callback(
            lambda response: self._goal_response_callback(response, epoch))

    def _goal_response_callback(self, future, epoch) -> None:
        """Track acceptance of the most recently submitted Nav2 goal."""
        self._goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:  # Action transport exceptions vary by RMW.
            self.get_logger().error(f'Failed to send Nav2 goal: {error}')
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected the following goal')
            return
        self._active_goal_handle = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda response: self._goal_result_callback(response, goal_handle))
        # A goal submitted before loss can be accepted after reacquisition.
        if self._target_lost or epoch != self._goal_epoch:
            self._cancel_navigation(goal_handle)

    def _goal_result_callback(self, future, goal_handle) -> None:
        """Release navigation ownership only after an action has finished."""
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(f'Nav2 result unavailable: {error}')
            return
        self._canceling_goals = [
            handle for handle in self._canceling_goals
            if handle is not goal_handle]
        if self._active_goal_handle is goal_handle:
            self._active_goal_handle = None

    def _cancel_navigation(self, goal_handle) -> None:
        """Wait for navigation to finish before allowing search commands."""
        if any(handle is goal_handle for handle in self._canceling_goals):
            return
        self._canceling_goals.append(goal_handle)
        goal_handle.cancel_goal_async()
        if self._active_goal_handle is goal_handle:
            self._active_goal_handle = None

    def _remember_search_direction(self, pixel_x, image_width) -> None:
        """Turn toward the last observed side, ignoring small centre jitter."""
        offset = pixel_x - self._camera_intrinsics[2]
        if abs(offset) > image_width * 0.05:
            # Optical X points right, whereas positive base yaw turns left.
            self._search_direction = -1.0 if offset > 0.0 else 1.0

    def _check_target_timeout(self) -> None:
        """Refresh lost-target commands independently of incoming images."""
        if self._last_rgb_time is None:
            return
        self._handle_missing_detection()
        if (not self._target_lost or self._goal_request_pending
                or self._canceling_goals):
            return
        image_age = (
            self.get_clock().now() - self._last_rgb_time).nanoseconds / 1e9
        search_age = (
            self.get_clock().now() - self._last_detection_time
        ).nanoseconds / 1e9 - self._detection_timeout
        if (self._search_when_lost and self._has_seen_target
                and image_age < self._detection_timeout
                and search_age < self._search_duration):
            self._publish_search_command()
        else:
            # A stopped image stream cannot support visual reacquisition.
            self._cmd_vel_publisher.publish(Twist())

    def _handle_missing_detection(self) -> None:
        """Cancel stale navigation and apply the configured lost policy."""
        elapsed = (
            self.get_clock().now() - self._last_detection_time
        ).nanoseconds / 1e9
        if elapsed < self._detection_timeout:
            return

        if not self._target_lost:
            self._target_lost = True
            self._goal_epoch += 1
            if self._active_goal_handle is not None:
                self._cancel_navigation(self._active_goal_handle)

            if self._search_when_lost:
                self.get_logger().info(
                    'Human lost; bounded arc search pending navigation cancellation')
            else:
                self._cmd_vel_publisher.publish(Twist())
                self.get_logger().info('Human lost; holding position')

    def _publish_search_command(self) -> None:
        """Search along a forward arc within the chassis curvature limit."""
        command = Twist()
        command.linear.x = self._search_linear_speed
        command.angular.z = min(
            abs(self._search_angular_speed),
            self._search_linear_speed / self._min_turning_radius,
        ) * self._search_direction
        self._cmd_vel_publisher.publish(command)

    def _stop_search(self) -> None:
        """Clear the lost state when a person is reacquired."""
        if not self._target_lost:
            return
        self._cmd_vel_publisher.publish(Twist())
        self._target_lost = False
        self.get_logger().info('Human reacquired; resuming navigation')

    def _show_debug_frame(self, frame) -> None:
        """Display the debug image only when explicitly enabled."""
        if not self._show_debug_view:
            return
        cv2.imshow('Human Follower', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        """Release native detector and GUI resources before shutdown."""
        if self._pose_detector is not None:
            self._pose_detector.close()
        if self._show_debug_view:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the human follower node."""
    rclpy.init(args=args)
    node = None
    try:
        node = HumanFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
