#!/usr/bin/env python3

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from autocar_msgs.msg import State2D


class Localisation(Node):

    def __init__(self):

        super().__init__('localisation')

        # Initialise publishers
        self.localisation_pub = self.create_publisher(State2D, '/autocar/state2D', 10)

        # TF broadcaster: publishes odom -> base_link so RViz and the TF tree
        # can locate the robot.  Without this nothing downstream can resolve
        # transforms from the robot frame.
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(Odometry, '/autocar/odom', self.vehicle_state_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', 50.0)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)

        except Exception:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None

    def vehicle_state_cb(self, msg):
        self.state = msg
        self.update_state()

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Broadcast odom -> base_link TF so the entire robot TF tree is
        # connected.  Use the original quaternion from the Gazebo odometry
        # message directly — no round-trip through yaw — and timestamp with
        # the incoming message header for sim-time consistency.
        t = TransformStamped()
        t.header.stamp = self.state.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.state.pose.pose.position.x
        t.transform.translation.y = self.state.pose.pose.position.y
        t.transform.translation.z = self.state.pose.pose.position.z
        t.transform.rotation = self.state.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.pose.position.x
        state2d.pose.y = self.state.pose.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)
        # Yaw stays in [-pi, pi] to match spline output convention

        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.twist.linear.x
        state2d.twist.y = self.state.twist.twist.linear.y
        state2d.twist.w = self.state.twist.twist.angular.z

        self.localisation_pub.publish(state2d)

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        localisation = Localisation()

        # Stop the node from exiting
        rclpy.spin(localisation)

    finally:
        localisation.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()