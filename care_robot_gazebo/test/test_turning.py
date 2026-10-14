"""Verify actual chassis motion, not just wheel-integrated odometry."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.model_pb2 import Model
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


def yaw(orientation):
    """Read planar yaw from a Gazebo quaternion."""
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y ** 2 + q.z ** 2))


def angle_change(before, after):
    """Return the signed shortest angle change."""
    return math.atan2(math.sin(after - before), math.cos(after - before))


def physics_world(destination):
    """Convert the real URDF and remove rendering for a lightweight test."""
    urdf = (Path(__file__).resolve().parents[2]
            / 'care_robot_description' / 'urdf' / 'care_robot.urdf')
    converted = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf)], check=True,
        capture_output=True, text=True, timeout=10)
    model = ET.fromstring(converted.stdout).find('model')
    ET.SubElement(model, 'pose').text = '0 0 0.06 0 0 0'
    for link in model.findall('link'):
        for child in list(link):
            if child.tag in {'visual', 'sensor'}:
                link.remove(child)
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='turning_test')
    physics = ET.SubElement(world, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    for library, system in [('physics', 'Physics'),
                            ('scene-broadcaster', 'SceneBroadcaster')]:
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{library}-system',
                      name=f'gz::sim::systems::{system}')
    world.append(ET.fromstring(
        '<model name="ground"><static>true</static><link name="ground">'
        '<collision name="ground"><geometry><plane><normal>0 0 1</normal>'
        '<size>100 100</size></plane></geometry></collision></link></model>'))
    world.append(model)
    ET.ElementTree(sdf).write(destination)


def test_chassis_moves_and_turns_in_both_directions(tmp_path, monkeypatch):
    """Verify rolling turns, Ackermann angles, reversing and no pivot turn."""
    monkeypatch.setenv('GZ_PARTITION', f'care_turning_{uuid.uuid4().hex}')
    world_file = tmp_path / 'turning.sdf'
    physics_world(world_file)
    node = Node()
    state = {}

    def clock(message):
        state['time'] = message.sim.sec + message.sim.nsec / 1e9

    def poses(message):
        for pose in message.pose:
            if pose.name == 'care_robot':
                state['actual'] = (
                    pose.position.x, pose.position.y, yaw(pose.orientation))

    def odometry(message):
        pose = message.pose
        state['odom'] = (
            pose.position.x, pose.position.y, yaw(pose.orientation))

    node.subscribe(Clock, '/clock', clock)
    node.subscribe(Pose_V, '/world/turning_test/dynamic_pose/info', poses)
    node.subscribe(Odometry, '/model/care_robot/odometry', odometry)

    def joints(message):
        state['steering'] = {
            joint.name: joint.axis1.position for joint in message.joint
            if 'steering_joint' in joint.name}

    node.subscribe(Model, '/world/turning_test/model/care_robot/joint_state', joints)
    publisher = node.advertise('/model/care_robot/cmd_vel', Twist)

    def move(linear, angular, duration):
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        start = state['time']
        deadline = time.monotonic() + 12
        while state['time'] - start < duration:
            assert time.monotonic() < deadline, 'Simulation stopped advancing'
            publisher.publish(command)
            time.sleep(0.02)

    with (tmp_path / 'gazebo.log').open('w') as log:
        server = subprocess.Popen(
            ['gz', 'sim', '-r', '-s', str(world_file)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 12
            while not {'time', 'actual', 'odom'} <= state.keys():
                assert server.poll() is None, 'Gazebo exited during startup'
                assert time.monotonic() < deadline, 'Missing Gazebo telemetry'
                time.sleep(0.02)
            move(0.0, 0.0, 0.5)
            before = state['actual']
            move(0.15, 0.0, 2.0)
            move(0.0, 0.0, 0.8)
            assert 0.2 < math.dist(before[:2], state['actual'][:2]) < 0.4

            # Steering at rest must not rotate the car body.
            before = state['actual']
            move(0.0, 0.4, 1.0)
            assert abs(angle_change(before[2], state['actual'][2])) < 0.03
            assert math.dist(before[:2], state['actual'][:2]) < 0.02
            move(0.0, 0.0, 0.8)

            for linear, direction in ((0.2, 1), (0.2, -1), (-0.2, 1)):
                before_actual, before_odom = state['actual'], state['odom']
                # Request tighter curvature than allowed to exercise the limit.
                move(linear, direction * 0.4, 3.0)
                steering = state['steering']
                left = steering['front_left_steering_joint']
                right = steering['front_right_steering_joint']
                turn_sign = direction * (1 if linear > 0 else -1)
                assert turn_sign * left > 0 and turn_sign * right > 0
                inner, outer = (left, right) if turn_sign > 0 else (right, left)
                assert abs(inner) > abs(outer) + 0.05
                # Both front wheels must intersect the same instantaneous centre.
                left_radius = 0.3588 / math.tan(left) + 0.19992 / 2
                right_radius = 0.3588 / math.tan(right) - 0.19992 / 2
                assert abs(left_radius - right_radius) < 0.03
                assert 0.63 < abs(left_radius) < 0.68
                move(0.0, 0.0, 0.8)
                actual = angle_change(before_actual[2], state['actual'][2])
                estimated = angle_change(before_odom[2], state['odom'][2])
                chord = math.dist(before_actual[:2], state['actual'][:2])
                radius = chord / (2 * math.sin(abs(actual) / 2))
                print(f'v={linear} direction={direction}: actual_yaw={actual:.3f}, '
                      f'odom_yaw={estimated:.3f}, radius={radius:.3f}, '
                      f'steering=({left:.3f}, {right:.3f})')
                assert 0.6 < direction * actual < 1.1
                assert 0.60 < radius < 0.85
                assert abs(actual - estimated) < 0.1, (
                    f'Wheel odometry diverged: actual={actual}, odom={estimated}')
        finally:
            publisher.publish(Twist())
            if server.poll() is None:
                os.killpg(server.pid, signal.SIGINT)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)
