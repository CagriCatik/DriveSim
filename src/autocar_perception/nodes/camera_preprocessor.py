#!/usr/bin/env python3

import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraPreprocessor(Node):
    def __init__(self):
        super().__init__('camera_preprocessor')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('raw_image_topic', '/autocar/camera/front/image_raw'),
                ('processed_image_topic', '/autocar/camera/front/image_proc'),
                ('camera_info_topic', '/autocar/camera/front/camera_info'),
                ('camera_frame', 'front_camera_link'),
                ('horizontal_fov', 1.3962634),
                ('force_frame_id', True),
            ],
        )

        raw_image_topic = str(self.get_parameter('raw_image_topic').value)
        processed_image_topic = str(self.get_parameter('processed_image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.horizontal_fov = float(self.get_parameter('horizontal_fov').value)
        self.force_frame_id = bool(self.get_parameter('force_frame_id').value)

        self.image_sub = self.create_subscription(Image, raw_image_topic, self.image_cb, 10)
        self.image_pub = self.create_publisher(Image, processed_image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)

    def image_cb(self, msg):
        image = deepcopy(msg)
        if self.force_frame_id or not image.header.frame_id:
            image.header.frame_id = self.camera_frame

        info = self.camera_info_from_image(image)
        self.image_pub.publish(image)
        self.camera_info_pub.publish(info)

    def camera_info_from_image(self, image):
        info = CameraInfo()
        info.header = image.header
        info.width = image.width
        info.height = image.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = self.focal_length_px(image.width, self.horizontal_fov)
        fy = fx
        cx = (float(image.width) - 1.0) * 0.5
        cy = (float(image.height) - 1.0) * 0.5

        info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    @staticmethod
    def focal_length_px(width, horizontal_fov):
        if width <= 0 or horizontal_fov <= 0.0:
            return 0.0
        return float(width) / (2.0 * math.tan(horizontal_fov * 0.5))


def main(args=None):
    rclpy.init(args=args)
    node = CameraPreprocessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
