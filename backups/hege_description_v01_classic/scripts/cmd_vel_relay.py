#!/usr/bin/env python3
"""Safe simulation relay: /cmd_vel -> Ackermann controller reference.

Negative linear.x is intentionally allowed for reverse motion. A first-order
lag approximates drivetrain response; calibrate tau and limits from real tests.
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/ackermann_steering_controller/reference')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('drive_time_constant', 0.30)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_yaw_rate', 0.11)
        self.declare_parameter('max_linear_acceleration', 0.25)
        self.declare_parameter('max_yaw_acceleration', 0.20)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.timeout = float(self.get_parameter('command_timeout').value)
        self.tau = float(self.get_parameter('drive_time_constant').value)
        self.v_max = float(self.get_parameter('max_linear_speed').value)
        self.w_max = float(self.get_parameter('max_yaw_rate').value)
        self.a_max = float(self.get_parameter('max_linear_acceleration').value)
        self.alpha_max = float(self.get_parameter('max_yaw_acceleration').value)

        self.target_v = 0.0
        self.target_w = 0.0
        self.output_v = 0.0
        self.output_w = 0.0
        self.last_command = None
        self.last_update = self.get_clock().now()

        self.pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.on_command, 10)
        self.create_timer(0.02, self.update)  # 50 Hz
        self.get_logger().info(
            f'{self.input_topic} -> {self.output_topic}; reverse enabled; tau={self.tau:.2f}s')

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def on_command(self, msg):
        self.target_v = self.clamp(msg.linear.x, -self.v_max, self.v_max)
        self.target_w = self.clamp(msg.angular.z, -self.w_max, self.w_max)
        self.last_command = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = max(1e-4, (now - self.last_update).nanoseconds * 1e-9)
        self.last_update = now

        stale = self.last_command is None or (now - self.last_command).nanoseconds * 1e-9 > self.timeout
        desired_v = 0.0 if stale else self.target_v
        desired_w = 0.0 if stale else self.target_w

        lag = 1.0 if self.tau <= 0.0 else dt / (self.tau + dt)
        lag_v = self.output_v + lag * (desired_v - self.output_v)
        lag_w = self.output_w + lag * (desired_w - self.output_w)
        self.output_v += self.clamp(lag_v - self.output_v, -self.a_max * dt, self.a_max * dt)
        self.output_w += self.clamp(lag_w - self.output_w, -self.alpha_max * dt, self.alpha_max * dt)

        if stale and math.isclose(self.output_v, 0.0, abs_tol=1e-3):
            self.output_v = 0.0
        if stale and math.isclose(self.output_w, 0.0, abs_tol=1e-3):
            self.output_w = 0.0

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = self.output_v
        msg.twist.angular.z = self.output_w
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
