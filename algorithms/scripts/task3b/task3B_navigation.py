#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 3B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          4372
# Author List:		Jinadev R P
# Filename:		    task3B_navigation.py
# Functions:
#			        wrap_angle, yaw_from_quaternion, scan_to_base_points
# Classes:          ObstacleGuard, PathFollower, FollowPhase, ebot_navigator
# Nodes:		    ebot_navigator_node
#                   Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan, /tf_static ]


################### IMPORT MODULES #######################

import math
import sys
from collections import deque
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


##################### TASK CONSTANTS #######################

path_topic = '/ebot_path'
odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'

body_frame = 'base_footprint'
lidar_frame = 'laser'

# What /cmd_vel accepts. The base clips anything beyond these.
max_linear_mps = 0.5
max_angular_rps = 1.0

# eBot footprint (eBot manual): 0.62 m long, 0.49 m wide.
half_length_m = 0.31
half_width_m = 0.245

# --- Following the path ---
cruise_mps = 0.35
turn_first_rad = 0.35                         # a path starts with a turn on the spot when it begins facing this far off it
turn_done_rad = 0.10                          # the turn on the spot ends inside this
lookahead_min_m = 0.45                        # how far ahead on the path the eBot aims
lookahead_gain_s = 0.8                        # extra lookahead per m/s of speed
turn_in_place_rad = 0.8                       # heading error above this: stop and turn
heading_kp = 1.6
min_turn_rps = 0.25                           # smallest turn command that overcomes static friction
heading_deadband_rad = 0.03
progress_window = 30                          # path points searched ahead (3 m) when finding the eBot's place
arrival_radius_m = 0.08                       # inside this of the end point the eBot stops and turns to the end yaw
arrival_slow_m = 0.8                          # slow down inside this distance of the end point
arrival_min_mps = 0.1                         # the base does not move below roughly this drive speed, so the approach never asks for less
settle_radius_m = 0.20                        # stuck inside this of the end point (and the task allows 0.3 m): settle here
stall_speed_mps = 0.01                        # measured speed below this counts as not moving
stall_cycles = 20                             # control cycles (1 s) of not moving before settling
guard_release_m = 0.70                        # inside this of the end point the lidar layer stands down: the planner has already cleared the pose
align_tolerance_rad = 0.04                    # the end yaw is reached inside this
speed_window = 8                              # odometry speeds averaged: a single reading carries noise of about 0.02 m/s

# --- Lidar safety layer ---
corridor_margin_m = 0.04                      # clearance kept beside the body
slow_range_m = 0.60                           # start slowing this far beyond the front edge
stop_gap_m = 0.12                             # stop and turn away inside this gap
avoid_turn_rps = 0.5
turn_toward_path_rad = 0.15                   # stopped by a rock: turn back to the path when facing this far off it


##################### HELPER FUNCTIONS #######################

def wrap_angle(angle):
    '''
    Description:    Wrap an angle to the range -pi..pi, so a turn never goes the long way round.
    '''

    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    '''
    Description:    Heading about the vertical axis of an orientation quaternion.
    '''

    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_to_base_points(scan, lidar_x, lidar_y, lidar_yaw):
    '''
    Description:    Valid lidar returns as (x, y) points in the body frame (x forward).
                    Rays with no return (inf) or outside range_min..range_max are dropped.
    '''

    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    ranges, angles = ranges[valid], angles[valid] + lidar_yaw
    return np.column_stack((lidar_x + ranges * np.cos(angles),
                            lidar_y + ranges * np.sin(angles)))


##################### CLASS DEFINITION #######################

class ObstacleGuard:
    '''
    ___CLASS___

    Description:    Safety layer between the path follower and the wheels. It looks at the
                    corridor the body is about to sweep: anything in it slows the eBot down,
                    and anything too close stops it and turns it away.
    '''

    def limit(self, linear, angular, obstacles, heading_error):
        '''
        Args:
            linear, angular (float):    the follower's wish
            obstacles (ndarray):        lidar points in the body frame, shape (n, 2)
            heading_error (float):      how far the eBot faces from the path, in radians

        Returns:
            (linear, angular) after the safety layer.
        '''

        if linear <= 0.0 or len(obstacles) == 0:
            return linear, angular

        ahead = obstacles[(obstacles[:, 0] > half_length_m)
                          & (obstacles[:, 0] < half_length_m + slow_range_m)
                          & (np.abs(obstacles[:, 1]) < half_width_m + corridor_margin_m)]
        if len(ahead) == 0:
            return linear, angular

        gap = float(ahead[:, 0].min()) - half_length_m
        if gap > stop_gap_m + 0.05:
            scale = (gap - stop_gap_m) / (slow_range_m - stop_gap_m)
            return linear * min(max(scale, 0.0), 1.0), angular

        if abs(heading_error) > turn_toward_path_rad:
            return 0.0, angular                   # the path itself is clear: first face it, standing still
        side = float(np.median(ahead[:, 1]))
        return 0.0, -math.copysign(avoid_turn_rps, side if abs(side) > 1.0e-3 else 1.0)


class FollowPhase(Enum):
    '''Where the follower is on its path.'''

    TURN = 0              # start of a path: turn on the spot until facing along it
    TRACK = 1             # drive along the path
    ALIGN = 2             # on the end point: turn on the spot to the end yaw
    DONE = 3              # stopped on the end pose


class PathFollower:
    '''
    ___CLASS___

    Description:    Follows one path with pure pursuit, then turns on the spot to the path's
                    end yaw. It holds no ROS objects, so it can be tested on its own.
    '''

    def __init__(self, points, end_yaw, guard=None):
        self.points = np.asarray(points, dtype=float)
        self.end = self.points[-1]
        self.end_yaw = end_yaw
        self.guard = guard or ObstacleGuard()
        self.phase = FollowPhase.TURN
        self.progress = 0                         # index of the path point nearest the eBot
        self.turning = False                      # a turn on the spot has begun and runs on to turn_done_rad
        self.stalled_cycles = 0                   # consecutive cycles spent close to the end without moving

    @property
    def done(self):
        return self.phase is FollowPhase.DONE

    def command(self, pose, speed, obstacles):
        '''
        Description:    One control step.

        Args:
            pose (tuple):           (x, y, yaw) of the body in the map frame
            speed (float):          measured forward speed (m/s)
            obstacles (ndarray):    lidar points in the body frame, shape (n, 2)

        Returns:
            (linear m/s, angular rad/s), inside the base's limits.
        '''

        x, y, yaw = pose
        if self.phase is FollowPhase.DONE:
            return 0.0, 0.0

        if self.phase is FollowPhase.TURN:
            turn = self._start_turn(x, y, yaw)
            if turn is not None:
                return turn

        to_end = math.hypot(self.end[0] - x, self.end[1] - y)
        if self.phase is FollowPhase.TRACK and (to_end <= arrival_radius_m
                                                or self._is_stalled_near_end(to_end, speed)):
            self.phase = FollowPhase.ALIGN

        if self.phase is FollowPhase.ALIGN:
            return self._align(yaw)

        self._advance_progress(x, y)
        aim_x, aim_y = self._aim_point(speed)
        error = wrap_angle(math.atan2(aim_y - y, aim_x - x) - yaw)

        linear = cruise_mps * max(0.0, 1.0 - abs(error) / turn_in_place_rad)
        linear = min(linear, max(arrival_min_mps, 0.5 * to_end)) if to_end < arrival_slow_m else linear
        angular = self._steer(error)

        if to_end > guard_release_m:
            linear, angular = self.guard.limit(linear, angular, obstacles, error)
        return self._clip(linear, angular)

    def _start_turn(self, x, y, yaw):
        '''
        Turn on the spot towards the first stretch of the path, so the eBot never sets off
        sideways. Returns the command, or None once it faces the path (the phase becomes TRACK).
        '''
        aim_x, aim_y = self._aim_point(0.0)
        error = wrap_angle(math.atan2(aim_y - y, aim_x - x) - yaw)
        if abs(error) <= (turn_done_rad if self.turning else turn_first_rad):
            self.phase = FollowPhase.TRACK
            return None
        self.turning = True
        return self._clip(0.0, self._steer(error, 0.0))

    def _is_stalled_near_end(self, to_end, speed):
        '''True once the eBot has stopped moving close enough to the end point to settle there.'''
        near_and_still = to_end <= settle_radius_m and abs(speed) < stall_speed_mps
        self.stalled_cycles = self.stalled_cycles + 1 if near_and_still else 0
        return self.stalled_cycles >= stall_cycles

    def _align(self, yaw):
        '''Turn on the spot to the end yaw and stop there.'''
        error = wrap_angle(self.end_yaw - yaw)
        if abs(error) <= align_tolerance_rad:
            self.phase = FollowPhase.DONE
            return 0.0, 0.0
        return self._clip(0.0, self._steer(error, 0.0))

    @staticmethod
    def _steer(error, deadband=heading_deadband_rad):
        '''Turn rate for a heading error: proportional, never below what moves the wheels.'''
        if abs(error) <= deadband:
            return 0.0
        rate = heading_kp * error
        return math.copysign(max(abs(rate), min_turn_rps), rate)

    @staticmethod
    def _clip(linear, angular):
        '''Keep a command inside what /cmd_vel accepts.'''
        return (min(max(linear, -max_linear_mps), max_linear_mps),
                min(max(angular, -max_angular_rps), max_angular_rps))

    def _advance_progress(self, x, y):
        '''Move the progress marker to the nearest path point just ahead of it.'''
        low = self.progress
        high = min(len(self.points), low + progress_window)
        gaps = np.hypot(self.points[low:high, 0] - x, self.points[low:high, 1] - y)
        self.progress = low + int(np.argmin(gaps))

    def _aim_point(self, speed):
        '''The point on the path to steer at: a lookahead distance beyond the progress marker.'''
        reach = lookahead_min_m + lookahead_gain_s * speed
        steps = np.hypot(*np.diff(self.points[self.progress:], axis=0).T)
        index = self.progress + int(np.searchsorted(np.cumsum(steps), reach)) + 1
        return self.points[min(index, len(self.points) - 1)]


class ebot_navigator(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the path
                    published on /ebot_path, and turn it to the path's end yaw.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_navigator
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_navigator_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # The path is published once per step and held for late subscribers, so the
        # subscription must ask for the same durability.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, latched)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = Buffer()                                                       # where the lidar sits on the body
        self.tf_listener = TransformListener(self.tf_buffer, self)

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.path = None                                                                # the path to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())

        ############ ADD YOUR CODE HERE ############

        self.pose = None                                                                # (x, y, yaw) of the body
        self.speed = 0.0                                                                # measured forward speed, averaged
        self.speeds = deque(maxlen=speed_window)
        self.lidar_mount = None                                                         # (x, y, yaw) of the lidar on the body
        self.follower = None                                                            # PathFollower for the latest path
        self.path_count = 0
        self.failure_reported = False

        ############################################


    def pathcb(self, data):
        '''
        Description:    Callback function for the path topic.
                        Use this function to receive the path the eBot has to drive.

        Args:
            data (Path):    The path, as a sequence of poses

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if len(data.poses) < 2:
            self.get_logger().warn('Ignoring a path with fewer than two poses')
            return

        points = [(p.pose.position.x, p.pose.position.y) for p in data.poses]
        end_yaw = yaw_from_quaternion(data.poses[-1].pose.orientation)

        # The planner publishes a path only while the eBot stands still, so a new path
        # always replaces the old one.
        self.follower = PathFollower(points, end_yaw)
        self.path_count += 1
        self.get_logger().info(
            f'Path {self.path_count} received: {len(points)} points in frame '
            f'"{data.header.frame_id}", ending at ({points[-1][0]:.2f}, {points[-1][1]:.2f}) '
            f'yaw {end_yaw:.2f}')

        ############################################


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        position = data.pose.pose.position
        self.pose = (position.x, position.y, yaw_from_quaternion(data.pose.pose.orientation))
        self.speeds.append(data.twist.twist.linear.x)
        self.speed = float(np.mean(self.speeds))
        self.odom = data

        ############################################


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic.
                        Use this function to receive what the lidar currently sees.

        Args:
            data (LaserScan):    One lidar sweep

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        self.scan = data

        ############################################


    def read_lidar_mount(self):
        '''
        Description:    Where the lidar sits on the body, from /tf_static. Returns True once known.
        '''

        if self.lidar_mount is not None:
            return True

        try:
            tf = self.tf_buffer.lookup_transform(body_frame, lidar_frame, Time())
        except Exception:                                   # frames not published yet
            return False

        t = tf.transform.translation
        self.lidar_mount = (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))
        self.get_logger().info(
            f'Lidar mount: x={t.x:.3f} y={t.y:.3f} yaw={self.lidar_mount[2]:.3f}')
        return True


    def publish_velocity(self, linear, angular):
        '''Send one Twist; only linear.x and angular.z do anything on the eBot.'''
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)


    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the path.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        try:
            self.navigate()
        except Exception as error:                          # never leave the base coasting
            self.publish_velocity(0.0, 0.0)
            if not self.failure_reported:
                self.failure_reported = True
                self.get_logger().error(f'Navigation cycle failed, base stopped: {error!r}')

        ############################################


    def navigate(self):
        '''
        Description:    One control cycle: steer along the latest path, or stand still.
        '''

        if self.pose is None or self.scan is None or not self.read_lidar_mount():
            return
        follower = self.follower
        if follower is None or follower.done:
            self.publish_velocity(0.0, 0.0)
            return

        phase_before = follower.phase
        mount_x, mount_y, mount_yaw = self.lidar_mount
        obstacles = scan_to_base_points(self.scan, mount_x, mount_y, mount_yaw)
        linear, angular = follower.command(self.pose, self.speed, obstacles)
        self.publish_velocity(linear, angular)

        if follower.phase is not phase_before:
            self.get_logger().info(f'Path {self.path_count}: {follower.phase.name}')


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_navigator class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_navigator_process')              # creating ROS node

    node.get_logger().info('Node created: eBot navigator process')  # logging information

    ebot_navigator_class = ebot_navigator()                         # creating a new object for class 'ebot_navigator'

    rclpy.spin(ebot_navigator_class)                                # spining on the object to make it alive in ROS 2 DDS

    ebot_navigator_class.destroy_node()                             # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
