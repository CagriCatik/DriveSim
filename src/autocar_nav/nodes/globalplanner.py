#!/usr/bin/env python3

import os

import numpy as np
import pandas as pd
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Pose2D, PoseArray
from rclpy.node import Node

from autocar_msgs.msg import Path2D, State2D


class GlobalPathPlanner(Node):

    def __init__(self):

        ''' Class constructor to initialise the class '''

        super().__init__('global_planner')

        # Initialise publisher(s)
        self.goals_pub = self.create_publisher(Path2D, '/autocar/goals', 10)
        self.goals_viz_pub = self.create_publisher(PoseArray, '/autocar/viz_goals', 10)

        # Initialise subscriber(s)
        self.localisation_sub = self.create_subscription(
            State2D, '/autocar/state2D', self.vehicle_state_cb, 10
        )

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', 2.0),
                    ('waypoints_ahead', 3),
                    ('waypoints_behind', 2),
                    ('passed_threshold', 0.25),
                    ('waypoints', 0),
                    ('centreofgravity_to_frontaxle', 1.483)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)
            self.wp_ahead = int(self.get_parameter("waypoints_ahead").value)
            self.wp_behind = int(self.get_parameter("waypoints_behind").value)
            self.passed_threshold = float(self.get_parameter("passed_threshold").value)
            self.cg2frontaxle = float(self.get_parameter("centreofgravity_to_frontaxle").value)

        except Exception:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Get path to waypoints.csv
        dir_path = os.path.join(
            get_package_share_directory('autocar_nav'), 'data', 'waypoints.csv'
        )
        df = pd.read_csv(dir_path)

        # Import waypoints.csv into class variables ax and ay
        self.ax = df['X-axis'].values.tolist()
        self.ay = df['Y-axis'].values.tolist()

        # Class constants
        self.waypoints = min(len(self.ax), len(self.ay))
        self.wp_published = self.wp_ahead + self.wp_behind

        # Class variables
        self.x = None
        self.y = None
        self.theta = None
        # Dirty flag: True when new state has arrived and we should recompute
        self._state_updated = False

        # Timer runs at configured frequency (default 2 Hz) — Bug 9 fix
        dt = 1.0 / self.frequency
        self.timer = self.create_timer(dt, self.timer_cb)

    def timer_cb(self):
        '''Periodic callback that recomputes waypoints only when state has changed.'''
        if self._state_updated and self.x is not None:
            self._state_updated = False
            self.set_waypoints()

    def vehicle_state_cb(self, msg):
        '''
        Callback function to update vehicle state.
        Only sets a dirty flag; computation happens in the timer callback.
        '''
        self.x = msg.pose.x
        self.y = msg.pose.y
        self.theta = msg.pose.theta
        self._state_updated = True

    def set_waypoints(self):
        '''
        Determines the appropriate set of waypoints to publish by:
        1. Identifying waypoint closest to front axle
        2. Determining if this point is ahead or behind via transformation
        3. Preserving a fixed window of points ahead/behind the vehicle
        '''

        # Position of vehicle front axle. The base_link convention is +X forward.
        fx = self.x + self.cg2frontaxle * np.cos(self.theta)
        fy = self.y + self.cg2frontaxle * np.sin(self.theta)

        dx = [fx - icx for icx in self.ax]
        dy = [fy - icy for icy in self.ay]

        d = np.hypot(dx, dy)
        closest_id = int(np.argmin(d))

        transform = self.frame_transform(
            self.ax[closest_id], self.ay[closest_id], fx, fy, self.theta
        )

        if closest_id < 2:
            self.get_logger().info('Closest Waypoint #{} (Starting Path)'.format(closest_id))
            px = self.ax[0: self.wp_published]
            py = self.ay[0: self.wp_published]

        elif closest_id > (self.waypoints - self.wp_published):
            self.get_logger().info('Closest Waypoint #{} (Terminating Path)'.format(closest_id))
            px = self.ax[-self.wp_published:]
            py = self.ay[-self.wp_published:]

        elif transform[0] < (0.0 - self.passed_threshold):
            self.get_logger().info('Closest Waypoint #{} (Passed)'.format(closest_id))
            px = self.ax[closest_id - (self.wp_behind - 1): closest_id + (self.wp_ahead + 1)]
            py = self.ay[closest_id - (self.wp_behind - 1): closest_id + (self.wp_ahead + 1)]

        else:
            self.get_logger().info('Closest Waypoint #{} (Approaching)'.format(closest_id))
            px = self.ax[(closest_id - self.wp_behind): (closest_id + self.wp_ahead)]
            py = self.ay[(closest_id - self.wp_behind): (closest_id + self.wp_ahead)]

        self.publish_goals(px, py)

    def start_end_condition(self, closest_id):
        ''' [NOT IN USE] Dictates the goals published near the start / end of the waypoints list '''

        if closest_id < self.wp_behind:
            px = self.ax[0: self.wp_ahead]
            py = self.ay[0: self.wp_ahead]
            return px, py

        elif closest_id > self.waypoints - 1:
            px = self.ax[(self.waypoints - self.wp_ahead - 1): self.waypoints]
            py = self.ay[(self.waypoints - self.wp_ahead - 1): self.waypoints]
            return px, py

    def frame_transform(self, point_x, point_y, axle_x, axle_y, theta):
        '''
        Transforms the closest waypoint from world frame to vehicle frame.
        '''
        c = np.cos(-theta)
        s = np.sin(-theta)
        R = np.array(((c, -s), (s, c)))

        p = np.array((point_x, point_y))
        v = np.array((axle_x, axle_y))
        vp = p - v
        return R.dot(vp)

    def publish_goals(self, px, py):
        ''' Publishes an array of waypoints for the Local Path Planner '''

        waypoints = min(len(px), len(py))
        goals = Path2D()

        viz_goals = PoseArray()
        viz_goals.header.frame_id = "odom"
        viz_goals.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, waypoints):
            goal = Pose2D()
            goal.x = px[i]
            goal.y = py[i]
            goals.poses.append(goal)

            vpose = Pose()
            vpose.position.x = px[i]
            vpose.position.y = py[i]
            vpose.position.z = 0.0
            vpose.orientation.w = 1.0
            viz_goals.poses.append(vpose)

        self.goals_pub.publish(goals)
        self.goals_viz_pub.publish(viz_goals)


def main(args=None):

    rclpy.init(args=args)

    try:
        global_planner = GlobalPathPlanner()
        rclpy.spin(global_planner)

    finally:
        global_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
