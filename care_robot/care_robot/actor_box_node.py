"""Synchronize a non-contact proxy with a scripted Gazebo Sim actor."""

from bisect import bisect_right
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


@dataclass(frozen=True)
class Waypoint:
    """A timed six-degree-of-freedom actor pose."""

    time: float
    pose: tuple[float, float, float, float, float, float]


def load_trajectory(
        world_file: str, actor_name: str) -> tuple[list[Waypoint], bool, float]:
    """Load one actor's timed trajectory from an SDF world."""
    root = ElementTree.parse(world_file).getroot()
    actor = next(
        (
            item for item in root.iter('actor')
            if item.get('name') == actor_name
        ),
        None,
    )
    if actor is None:
        raise ValueError(f'actor {actor_name!r} was not found')

    script = actor.find('script')
    trajectory = None if script is None else script.find('trajectory')
    if script is None or trajectory is None:
        raise ValueError(f'actor {actor_name!r} has no scripted trajectory')

    loop = _xml_bool(script.findtext('loop'), default=True)
    delay = float(script.findtext('delay_start', default='0.0'))
    waypoints = []
    for item in trajectory.findall('waypoint'):
        pose_values = [float(value) for value in item.findtext('pose').split()]
        if len(pose_values) != 6:
            raise ValueError('each actor waypoint pose must contain 6 values')
        waypoints.append(Waypoint(
            time=float(item.findtext('time')),
            pose=tuple(pose_values),
        ))

    waypoints.sort(key=lambda item: item.time)
    if len(waypoints) < 2:
        raise ValueError('the actor trajectory needs at least 2 waypoints')
    return waypoints, loop, delay


def interpolate_trajectory(
        waypoints: list[Waypoint], loop: bool, elapsed: float
) -> tuple[float, float, float, float, float, float]:
    """Interpolate a scripted actor trajectory at a simulation time."""
    first_time = waypoints[0].time
    last_time = waypoints[-1].time
    if elapsed <= first_time:
        return waypoints[0].pose
    if loop and last_time > first_time:
        elapsed = first_time + (elapsed - first_time) % (
            last_time - first_time)
    elif elapsed >= last_time:
        return waypoints[-1].pose

    times = [waypoint.time for waypoint in waypoints]
    index = min(bisect_right(times, elapsed) - 1, len(waypoints) - 2)
    start = waypoints[index]
    end = waypoints[index + 1]
    fraction = (elapsed - start.time) / (end.time - start.time)
    position = tuple(
        start.pose[axis] + fraction * (end.pose[axis] - start.pose[axis])
        for axis in range(3)
    )
    rotation = tuple(
        _interpolate_angle(start.pose[axis], end.pose[axis], fraction)
        for axis in range(3, 6)
    )
    return position + rotation


def proxy_center_z(
        actor_origin_z: float,
        actor_ground_reference_z: float,
        proxy_center_height: float,
) -> float:
    """Map an actor origin height to a ground-relative proxy centre."""
    return (
        actor_origin_z - actor_ground_reference_z + proxy_center_height
    )


def _interpolate_angle(start: float, end: float, fraction: float) -> float:
    """Interpolate an angle along its shortest wrapped path."""
    difference = (end - start + math.pi) % (2.0 * math.pi) - math.pi
    return start + fraction * difference


def _xml_bool(value: str | None, default: bool) -> bool:
    """Parse an SDF boolean value."""
    if value is None:
        return default
    return value.strip().lower() in {'1', 'true', 'yes'}


class ActorProxyFollower(Node):
    """Mirror a scripted actor path to a non-contact Gazebo Sim proxy."""

    def __init__(self) -> None:
        """Initialize the trajectory and Gazebo Sim set-pose bridge."""
        super().__init__('actor_proxy_follower')
        self.declare_parameter('actor_name', 'actor')
        self.declare_parameter('proxy_entity_name', 'actor_tracking_proxy')
        self.declare_parameter('world_file', '')
        self.declare_parameter(
            'set_pose_service', '/world/default/set_pose')
        self.declare_parameter('actor_ground_reference_z', 1.0)
        self.declare_parameter('proxy_center_height', 0.85)
        self.declare_parameter('update_rate', 20.0)

        actor_name = str(self.get_parameter('actor_name').value)
        self._proxy_entity_name = str(
            self.get_parameter('proxy_entity_name').value)
        world_file = str(self.get_parameter('world_file').value)
        service_name = str(self.get_parameter('set_pose_service').value)
        self._actor_ground_reference_z = float(
            self.get_parameter('actor_ground_reference_z').value)
        self._proxy_center_height = float(
            self.get_parameter('proxy_center_height').value)
        update_rate = float(self.get_parameter('update_rate').value)

        if not Path(world_file).is_file():
            raise ValueError(f'world_file is not readable: {world_file!r}')
        self._waypoints, self._loop, self._delay = load_trajectory(
            world_file, actor_name)

        self._client = self.create_client(SetEntityPose, service_name)
        self._request_pending = False
        self._update_timer = self.create_timer(
            1.0 / max(update_rate, 1.0), self._update_pose)
        self._service_timer = self.create_timer(
            2.0, self._report_service_status)

    def _report_service_status(self) -> None:
        """Report bridge readiness without blocking node construction."""
        if self._client.service_is_ready():
            self._service_timer.cancel()
            self.get_logger().info('Gazebo Sim set-pose service is ready')
            return
        self.get_logger().info('Waiting for Gazebo Sim set-pose service')

    def _update_pose(self) -> None:
        """Move the non-contact proxy to the actor's interpolated pose."""
        if not self._client.service_is_ready() or self._request_pending:
            return

        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._delay
        x, y, z, roll, pitch, yaw = interpolate_trajectory(
            self._waypoints, self._loop, elapsed)
        quaternion = _quaternion_from_euler(roll, pitch, yaw)

        request = SetEntityPose.Request()
        request.entity.name = self._proxy_entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = proxy_center_z(
            z,
            self._actor_ground_reference_z,
            self._proxy_center_height,
        )
        request.pose.orientation.x = quaternion[0]
        request.pose.orientation.y = quaternion[1]
        request.pose.orientation.z = quaternion[2]
        request.pose.orientation.w = quaternion[3]

        self._request_pending = True
        future = self._client.call_async(request)
        future.add_done_callback(self._request_complete)

    def _request_complete(self, future) -> None:
        """Clear the in-flight guard and report service errors."""
        self._request_pending = False
        try:
            response = future.result()
        except Exception as error:  # Service transport errors vary by RMW.
            self.get_logger().error(f'Set-pose request failed: {error}')
            return
        if not response.success:
            self.get_logger().warning('Gazebo Sim rejected a set-pose request')


def _quaternion_from_euler(
        roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert roll, pitch, yaw to an x-y-z-w quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main(args=None) -> None:
    """Run the actor proxy follower node."""
    rclpy.init(args=args)
    node = ActorProxyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
