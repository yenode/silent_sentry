#!/usr/bin/env python3
"""
EMCON Controller Node
=====================
Emission-Control-aware command arbitration for silent UGV operations.

Subscribes:
  /cmd_vel_raw        (geometry_msgs/Twist)  — raw commands from planner
  /emcon_state        (std_msgs/Bool)        — True = EMCON active (suppress)

Publishes:
  /cmd_vel            (geometry_msgs/Twist)  — arbitrated commands
  /emcon_status       (std_msgs/String)      — current mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class EmconController(Node):
    def __init__(self):
        super().__init__('emcon_controller')

        # Parameters
        self.declare_parameter('emcon_slowdown_factor', 0.3)
        self.declare_parameter('emcon_max_angular', 0.1)
        self.slowdown = self.get_parameter('emcon_slowdown_factor').value
        self.max_ang  = self.get_parameter('emcon_max_angular').value

        self._emcon_active = False
        self._last_cmd     = Twist()

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_raw', self._cmd_cb, 10)
        self.create_subscription(Bool,  '/emcon_state', self._emcon_cb, 10)

        # Publishers
        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',      10)
        self._status_pub = self.create_publisher(String, '/emcon_status', 10)

        self.get_logger().info('EMCON Controller ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _emcon_cb(self, msg: Bool):
        self._emcon_active = msg.data
        mode = 'EMCON_ACTIVE' if msg.data else 'EMCON_CLEAR'
        self.get_logger().info(f'EMCON state changed: {mode}')
        self._status_pub.publish(String(data=mode))

    def _cmd_cb(self, msg: Twist):
        out = Twist()
        if self._emcon_active:
            # Suppress: slow down, limit turns, no aggressive manoeuvres
            out.linear.x  = msg.linear.x  * self.slowdown
            out.angular.z = max(-self.max_ang,
                                min(self.max_ang, msg.angular.z * self.slowdown))
        else:
            out = msg
        self._cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EmconController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
