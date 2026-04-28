#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from autocar_msgs.msg import DetectedObjectArray, Path2D, Trajectory, TrajectoryPoint
from autocar_planning.frenet_planner import FrenetPlanner
from autocar_planning.reference_path import ReferencePath


class FrenetPlannerNode(Node):
    def __init__(self):
        super().__init__('frenet_planner')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_frequency', 10.0),
                ('target_velocity', 3.0),
                ('lateral_offsets', [-1.0, -0.5, 0.0, 0.5, 1.0]),
                ('speed_factors', [0.6, 1.0, 1.2]),
                ('collision_radius', 1.5),
                ('planning_frame', 'odom'),
            ],
        )

        self.frequency = float(self.get_parameter('update_frequency').value)
        self.target_velocity = float(self.get_parameter('target_velocity').value)
        self.planning_frame = str(self.get_parameter('planning_frame').value)
        self.reference_path = None
        self.objects = []

        self.planner = FrenetPlanner(
            list(self.get_parameter('lateral_offsets').value),
            list(self.get_parameter('speed_factors').value),
            float(self.get_parameter('collision_radius').value),
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(Path2D, '/autocar/path', self.path_cb, 10)
        self.objects_sub = self.create_subscription(
            DetectedObjectArray, '/autocar/objects', self.objects_cb, 10
        )
        self.target_vel_sub = self.create_subscription(
            Float64, '/autocar/target_velocity', self.target_vel_cb, 10
        )
        self.trajectory_pub = self.create_publisher(Trajectory, '/autocar/trajectory', 10)
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_cb)

    def path_cb(self, msg):
        self.reference_path = ReferencePath.from_path2d(msg)

    def objects_cb(self, msg):
        source_frame = msg.header.frame_id.strip().lstrip('/')
        if source_frame in ('', self.planning_frame):
            self.objects = list(msg.objects)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'Waiting for transform {source_frame} -> {self.planning_frame}: {exc}'
            )
            self.objects = []
            return

        self.objects = [self.transform_object(obj, transform) for obj in msg.objects]

    def transform_object(self, obj, transform):
        transformed = deepcopy(obj)
        x = obj.pose.position.x
        y = obj.pose.position.y
        z = obj.pose.position.z
        tx, ty, tz = self.transform_point(x, y, z, transform)
        transformed.pose.position.x = tx
        transformed.pose.position.y = ty
        transformed.pose.position.z = tz
        return transformed

    @staticmethod
    def transform_point(x, y, z, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w

        rx = (1.0 - 2.0 * (qy * qy + qz * qz)) * x
        rx += 2.0 * (qx * qy - qz * qw) * y
        rx += 2.0 * (qx * qz + qy * qw) * z

        ry = 2.0 * (qx * qy + qz * qw) * x
        ry += (1.0 - 2.0 * (qx * qx + qz * qz)) * y
        ry += 2.0 * (qy * qz - qx * qw) * z

        rz = 2.0 * (qx * qz - qy * qw) * x
        rz += 2.0 * (qy * qz + qx * qw) * y
        rz += (1.0 - 2.0 * (qx * qx + qy * qy)) * z

        return (
            rx + translation.x,
            ry + translation.y,
            rz + translation.z,
        )

    def target_vel_cb(self, msg):
        self.target_velocity = float(msg.data)

    def timer_cb(self):
        if self.reference_path is None or self.reference_path.empty:
            return

        best = self.planner.plan(self.reference_path, self.target_velocity, self.objects)
        if best is None:
            return

        trajectory = Trajectory()
        trajectory.header.frame_id = self.planning_frame
        trajectory.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw, velocity, acceleration, curvature in zip(
            best.xs,
            best.ys,
            best.yaws,
            best.velocities,
            best.accelerations,
            best.curvatures,
        ):
            point = TrajectoryPoint()
            point.x = float(x)
            point.y = float(y)
            point.yaw = float(yaw)
            point.velocity = float(velocity)
            point.acceleration = float(acceleration)
            point.curvature = float(curvature)
            trajectory.points.append(point)

        self.trajectory_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
