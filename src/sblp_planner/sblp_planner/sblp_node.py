#!/usr/bin/env python3
"""
SBLP Planner Node
=================
Scenario-Based Local Planner.
Switches trajectory primitives based on terrain class from VLM costmap.

Terrain classes:
  0 = open_corridor  → sprint primitive (max speed)
  1 = sand_dune      → cautious S-curve primitive
  2 = rock_field     → stop-and-assess primitive

Subscribes:
  /terrain_class     (std_msgs/Int8)         — from vlm_costmap
  /goal_pose         (geometry_msgs/PoseStamped)

Publishes:
  /cmd_vel_raw       (geometry_msgs/Twist)   — to emcon_controller
  /sblp/scenario     (std_msgs/String)       — active scenario name
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist, PoseStamped
import math


SCENARIO_MAP = {
    0: 'open_corridor',
    1: 'sand_dune',
    2: 'rock_field',
}


class SBLPPlanner(Node):
    def __init__(self):
        super().__init__('sblp_planner')

        self.declare_parameter('max_linear_vel',  1.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self._max_lin = self.get_parameter('max_linear_vel').value
        self._max_ang = self.get_parameter('max_angular_vel').value

        self._terrain_class = 0
        self._goal: PoseStamped | None = None

        # Subscribers
        self.create_subscription(Int8,        '/terrain_class', self._terrain_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose',     self._goal_cb,    10)

        # Publishers
        self._cmd_pub      = self.create_publisher(Twist,  '/cmd_vel_raw',  10)
        self._scenario_pub = self.create_publisher(String, '/sblp/scenario', 10)

        # Planning loop at 10 Hz
        self.create_timer(0.1, self._plan_loop)
        self.get_logger().info('SBLP Planner ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _terrain_cb(self, msg: Int8):
        self._terrain_class = msg.data
        scenario = SCENARIO_MAP.get(msg.data, 'unknown')
        self._scenario_pub.publish(String(data=scenario))

    def _goal_cb(self, msg: PoseStamped):
        self._goal = msg

    # ── Planning loop ─────────────────────────────────────────────────────────
    def _plan_loop(self):
        cmd = Twist()
        if self._terrain_class == 0:   # open corridor — go fast
            cmd.linear.x = self._max_lin
        elif self._terrain_class == 1: # sand dune — cautious
            cmd.linear.x  = self._max_lin * 0.4
            cmd.angular.z = math.sin(self.get_clock().now().nanoseconds * 1e-9) * 0.3
        elif self._terrain_class == 2: # rock field — stop
            cmd.linear.x = 0.0
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SBLPPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
