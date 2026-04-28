#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from autocar_msgs.msg import DetectedObjectArray


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_frequency', 30.0),
                ('raw_cmd_topic', '/autocar/raw_cmd_vel'),
                ('safe_cmd_topic', '/autocar/cmd_vel'),
                ('objects_topic', '/autocar/objects'),
                ('max_speed', 20.0),
                ('max_steering_angle', 0.95),
                ('command_timeout', 0.5),
                ('stop_distance', 2.0),
                ('collision_width', 1.2),
            ],
        )

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering = float(self.get_parameter('max_steering_angle').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.collision_width = float(self.get_parameter('collision_width').value)

        raw_cmd_topic = str(self.get_parameter('raw_cmd_topic').value)
        safe_cmd_topic = str(self.get_parameter('safe_cmd_topic').value)
        objects_topic = str(self.get_parameter('objects_topic').value)

        self.raw_cmd = Twist()
        self.last_cmd_time = None
        self.emergency_stop = False

        self.raw_sub = self.create_subscription(Twist, raw_cmd_topic, self.raw_cmd_cb, 10)
        self.objects_sub = self.create_subscription(DetectedObjectArray, objects_topic, self.objects_cb, 10)
        self.safe_pub = self.create_publisher(Twist, safe_cmd_topic, 10)

        frequency = float(self.get_parameter('update_frequency').value)
        self.timer = self.create_timer(1.0 / frequency, self.timer_cb)

    def raw_cmd_cb(self, msg):
        self.raw_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def objects_cb(self, msg):
        self.emergency_stop = False
        for obj in msg.objects:
            x = obj.pose.position.x
            y = obj.pose.position.y
            distance = math.hypot(x, y)
            in_path = x > 0.0 and abs(y) <= self.collision_width
            if in_path and distance <= self.stop_distance:
                self.emergency_stop = True
                return

    def timer_cb(self):
        cmd = Twist()
        stale = self.last_cmd_time is None
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            stale = age > self.command_timeout

        if stale or self.emergency_stop:
            self.safe_pub.publish(cmd)
            return

        cmd.linear.x = self.clamp(self.raw_cmd.linear.x, -self.max_speed, self.max_speed)
        cmd.angular.z = self.clamp(self.raw_cmd.angular.z, -self.max_steering, self.max_steering)
        self.safe_pub.publish(cmd)

    @staticmethod
    def clamp(value, lower, upper):
        return min(max(value, lower), upper)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
