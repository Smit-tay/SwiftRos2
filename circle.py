#!/usr/bin/env python3
# circle.py — circular motion via persistent ROS2 client.
# Copyright 2026 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Stays connected for the whole loop, avoiding per-call CLI overhead.
# Stops at the first move the firmware refuses.

import math
import sys
import rclpy
from rclpy.node import Node
from swiftpro_resources.srv import MoveTo

CENTRE_X    = 220.0
CENTRE_Y    = 0.0
Z           = 150.0
RADIUS      = 60.0
WAYPOINTS   = 36
SPEED       = 15.0      # F: 1 F = 80 deg/min of joint-space speed, so 20 deg/s
LOOPS       = 3


class CircleClient(Node):
    def __init__(self):
        super().__init__('circle_client')
        self.move_cli = self.create_client(MoveTo, '/swiftpro/move_to')

        if not self.move_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service move_to not available')
            sys.exit(1)

    def move_to(self, x, y, z, speed):
        req = MoveTo.Request()
        req.x, req.y, req.z, req.speed = x, y, z, speed
        fut = self.move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        r = fut.result()
        if not r.success:
            self.get_logger().error(f'move to ({x:.1f}, {y:.1f}, {z:.1f}): '
                                    f'{r.error_code} {r.message}')
            sys.exit(1)


def main():
    rclpy.init()
    node = CircleClient()

    node.get_logger().info(f'Moving to start position ({CENTRE_X + RADIUS}, 0, {Z})')
    node.move_to(CENTRE_X + RADIUS, CENTRE_Y, Z, SPEED)

    node.get_logger().info(f'Tracing circle: r={RADIUS}mm, {WAYPOINTS} waypoints × {LOOPS} loops')
    total_waypoints = WAYPOINTS * LOOPS
    for i in range(1, total_waypoints + 1):
        theta = (i % WAYPOINTS) * 2 * math.pi / WAYPOINTS
        x = CENTRE_X + RADIUS * math.cos(theta)
        y = CENTRE_Y + RADIUS * math.sin(theta)
        node.move_to(x, y, Z, SPEED)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
