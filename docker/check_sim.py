#!/usr/bin/env python3
"""Check live simulation data and Nav2 readiness from inside the container."""

import argparse
import sys
import time

from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('container_simulation_check')
    seen = set()
    topics = {
        '/clock': Clock,
        '/odom': Odometry,
        '/scan': LaserScan,
        '/robot_camera_sensor/image': Image,
        '/robot_camera_sensor/depth_image': Image,
    }
    subscriptions = [
        node.create_subscription(
            message_type, topic,
            lambda message, key=topic: seen.add(key), qos_profile_sensor_data)
        for topic, message_type in topics.items()
    ]
    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', lambda message: seen.add('/map'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    clients = {
        name: node.create_client(GetState, f'/{name}/get_state')
        for name in ('amcl', 'controller_server', 'planner_server',
                     'bt_navigator', 'collision_monitor')
    }
    pending = {}
    required = set(topics) | {'/map', 'map -> base_link'} | set(clients)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and seen != required:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            seen.add('map -> base_link')
        except TransformException:
            pass
        for name, client in clients.items():
            if name in seen:
                continue
            if name in pending and pending[name].done():
                result = pending.pop(name).result()
                if result is not None and result.current_state.label == 'active':
                    seen.add(name)
            if name not in seen and name not in pending and client.service_is_ready():
                pending[name] = client.call_async(GetState.Request())
    for item in sorted(required):
        print(f'{"OK" if item in seen else "MISSING"}: {item}')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if seen == required else 1


if __name__ == '__main__':
    sys.exit(main())
