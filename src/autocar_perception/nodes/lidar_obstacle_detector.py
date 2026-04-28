#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from autocar_msgs.msg import DetectedObject, DetectedObjectArray


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', '/autocar/scan'),
                ('objects_topic', '/autocar/objects'),
                ('max_detection_range', 25.0),
                ('cluster_gap', 0.5),
                ('min_cluster_points', 3),
                ('object_frame', 'front_lidar_link'),
            ],
        )

        scan_topic = str(self.get_parameter('scan_topic').value)
        objects_topic = str(self.get_parameter('objects_topic').value)
        self.object_frame = str(self.get_parameter('object_frame').value)
        self.max_detection_range = float(self.get_parameter('max_detection_range').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.object_seq = 0

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.objects_pub = self.create_publisher(DetectedObjectArray, objects_topic, 10)

    def scan_cb(self, msg):
        points = self.scan_to_points(msg)
        clusters = self.cluster_points(points)

        out = DetectedObjectArray()
        out.header = msg.header
        if self.object_frame:
            out.header.frame_id = self.object_frame

        for cluster in clusters:
            obj = self.cluster_to_object(cluster)
            out.objects.append(obj)

        self.objects_pub.publish(out)

    def scan_to_points(self, msg):
        points = []
        angle = msg.angle_min
        for distance in msg.ranges:
            valid = (
                math.isfinite(distance)
                and msg.range_min <= distance <= min(msg.range_max, self.max_detection_range)
            )
            if valid:
                points.append((distance * math.cos(angle), distance * math.sin(angle)))
            else:
                points.append(None)
            angle += msg.angle_increment
        return points

    def cluster_points(self, points):
        clusters = []
        current = []
        last = None

        for point in points:
            if point is None:
                self.push_cluster(clusters, current)
                current = []
                last = None
                continue

            if last is not None and np.hypot(point[0] - last[0], point[1] - last[1]) > self.cluster_gap:
                self.push_cluster(clusters, current)
                current = []

            current.append(point)
            last = point

        self.push_cluster(clusters, current)
        return clusters

    def push_cluster(self, clusters, cluster):
        if len(cluster) >= self.min_cluster_points:
            clusters.append(cluster)

    def cluster_to_object(self, cluster):
        pts = np.asarray(cluster, dtype=float)
        centroid = np.mean(pts, axis=0)
        span = np.ptp(pts, axis=0)

        pose = Pose()
        pose.position.x = float(centroid[0])
        pose.position.y = float(centroid[1])
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        dimensions = Vector3()
        dimensions.x = max(float(span[0]), 0.2)
        dimensions.y = max(float(span[1]), 0.2)
        dimensions.z = 1.0

        obj = DetectedObject()
        obj.id = f'lidar_{self.object_seq}'
        obj.label = 'obstacle'
        obj.confidence = 1.0
        obj.pose = pose
        obj.dimensions = dimensions
        obj.twist = Twist()
        self.object_seq += 1
        return obj


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
