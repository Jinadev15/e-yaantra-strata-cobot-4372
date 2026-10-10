#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1C of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    path_follower.py
# Functions:
#			        wrap_angle, GridMap, astar, line_free, shortcut, resample, plan_route,
#			        scan_to_base_points, RouteFollower
# Nodes:		    ebot_nav_node
#                   Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /map, /odom, /scan, /tf_static ]


################### IMPORT MODULES #######################

import heapq
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


##################### TASK CONSTANTS #######################

path_topic = '/ebot_path'
map_topic = '/map'
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

# --- Planning on /map ---
occupied_threshold = 50                       # occupancy value from which a cell is a rock
planning_radii_m = (0.52, 0.46, 0.40, 0.32, 0.26)   # body clearance tried in order; 0.40 m just clears the footprint corners, more leaves a margin
endpoint_clearance_m = 0.30                   # cleared around the start so a rover parked beside a rock can leave
trim_radius_m = 0.20                          # a leg ends at the first route point this close to its waypoint (no dead-end spikes)
pass_offset_m = 0.34                          # a waypoint may be passed this close instead of exactly on it (scoring allows 0.5 m)
clearance_weight = 2.0                        # extra path cost next to rocks, so the route keeps away from them
clearance_range_m = 1.0                       # the extra cost fades to zero this far from a rock
path_spacing_m = 0.10                         # spacing of the points the rover follows

# --- Following the route ---
cruise_mps = 0.35
reach_radius_m = 0.28                         # a waypoint is done inside this (scoring needs 0.5 m, the accuracy bonus 0.3 m)
final_reach_m = 0.15                          # the last waypoint is approached closer, since the rover stops there
reach_slack_m = 0.08                          # a waypoint is never asked closer than the route itself gets, plus this
reach_cap_m = 0.45                            # but always inside the 0.5 m scoring distance
lookahead_min_m = 0.45                        # how far ahead on the route the rover aims
lookahead_gain_s = 0.8                        # extra lookahead per m/s of speed
turn_in_place_rad = 0.8                       # heading error above this: stop and turn
heading_kp = 1.6
min_turn_rps = 0.25                           # smallest turn command that overcomes static friction
heading_deadband_rad = 0.03
final_slow_m = 0.8                            # slow down inside this distance of the last waypoint
progress_window = 30                          # route points searched ahead (3 m) when finding the rover's place
replan_distance_m = 0.7                       # farther than this from the route: plan again
max_replans = 5

# --- Lidar safety layer ---
corridor_margin_m = 0.08                      # clearance kept beside the body
slow_range_m = 0.60                           # start slowing this far beyond the front edge
stop_gap_m = 0.12                             # stop and turn away inside this gap
avoid_turn_rps = 0.5
turn_toward_route_rad = 0.15                  # stopped by a rock: turn back to the route when facing this far off it


##################### HELPER FUNCTIONS #######################

def wrap_angle(angle):
    '''
    Description:    Wrap an angle to the range -pi..pi, so a turn never goes the long way round.
    '''

    return math.atan2(math.sin(angle), math.cos(angle))


class GridMap:
    '''
    ___CLASS___

    Description:    The /map occupancy grid, with the conversions between world and cell and
                    the distance from every cell to the nearest rock.
    '''

    def __init__(self, msg):
        info = msg.info
        self.resolution = info.resolution
        self.width = info.width
        self.height = info.height
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y

        data = np.asarray(msg.data, dtype=np.int16).reshape(self.height, self.width)
        self.occupied = data >= occupied_threshold
        self.unknown_cells = int(np.count_nonzero(data < 0))

        if self.occupied.any():
            self.rock_distance_m = distance_transform_edt(~self.occupied) * self.resolution
        else:
            self.rock_distance_m = np.full(self.occupied.shape, 1.0e3)

    def to_cell(self, x, y):
        '''World (x, y) to (column, row), clamped to the grid.'''
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return (min(max(col, 0), self.width - 1), min(max(row, 0), self.height - 1))

    def to_world(self, col, row):
        '''Centre of a (column, row) cell in world coordinates.'''
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def inside(self, x, y):
        '''True when the world point lies on the grid.'''
        col = (x - self.origin_x) / self.resolution
        row = (y - self.origin_y) / self.resolution
        return 0.0 <= col < self.width and 0.0 <= row < self.height

    def blocked_mask(self, radius_m, start_cell):
        '''
        Cells the rover centre may not enter: those within radius_m of a rock.
        Around the start the clearance is relaxed to the half-width of the body, so a rover
        parked beside a rock can still drive away without ever entering it.
        '''
        blocked = self.rock_distance_m <= radius_m
        half = int(math.ceil(endpoint_clearance_m / self.resolution))
        col, row = start_cell
        rows = slice(max(row - half, 0), row + half + 1)
        cols = slice(max(col - half, 0), col + half + 1)
        blocked[rows, cols] = self.rock_distance_m[rows, cols] <= half_width_m
        return blocked

    def pass_cell(self, blocked, waypoint):
        '''
        The free cell nearest to a waypoint, within pass_offset_m of it, or None.
        A waypoint beside a rock is then passed at a safe spot instead of squeezed onto.
        '''
        col, row = self.to_cell(*waypoint)
        half = int(math.ceil(pass_offset_m / self.resolution))
        row0, col0 = max(row - half, 0), max(col - half, 0)
        free_rows, free_cols = np.nonzero(~blocked[row0:row + half + 1, col0:col + half + 1])
        if len(free_rows) == 0:
            return None
        free_rows, free_cols = free_rows + row0, free_cols + col0
        world_x = self.origin_x + (free_cols + 0.5) * self.resolution
        world_y = self.origin_y + (free_rows + 0.5) * self.resolution
        gaps = np.hypot(world_x - waypoint[0], world_y - waypoint[1])
        best = int(np.argmin(gaps))
        if gaps[best] > pass_offset_m:
            return None
        return int(free_cols[best]), int(free_rows[best])

    def cost_multiplier(self):
        '''Per-cell path cost factor: 1 far from rocks, rising towards rocks.'''
        closeness = np.clip(1.0 - self.rock_distance_m / clearance_range_m, 0.0, 1.0)
        return 1.0 + clearance_weight * closeness


_neighbours = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
               (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
               (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)))


def astar(blocked, multiplier, start, goal):
    '''
    Description:    A* search on the grid, 8-connected, never cutting a blocked corner.

    Args:
        blocked (ndarray):      True where the rover centre may not go
        multiplier (ndarray):   Per-cell cost factor (>= 1)
        start, goal (tuple):    (column, row)

    Returns:
        list of (column, row) from start to goal, or None when there is no way through.
    '''

    height, width = blocked.shape
    goal_col, goal_row = goal
    best = {start: 0.0}
    parent = {}
    heap = [(math.hypot(goal_col - start[0], goal_row - start[1]), 0.0, start)]

    while heap:
        _, cost, cell = heapq.heappop(heap)
        if cost > best.get(cell, math.inf):
            continue
        if cell == goal:
            cells = [cell]
            while cell in parent:
                cell = parent[cell]
                cells.append(cell)
            return cells[::-1]

        col, row = cell
        for d_col, d_row, step in _neighbours:
            n_col, n_row = col + d_col, row + d_row
            if not (0 <= n_col < width and 0 <= n_row < height) or blocked[n_row, n_col]:
                continue
            if d_col and d_row and (blocked[row, n_col] or blocked[n_row, col]):
                continue
            n_cost = cost + step * multiplier[n_row, n_col]
            if n_cost < best.get((n_col, n_row), math.inf):
                best[(n_col, n_row)] = n_cost
                parent[(n_col, n_row)] = cell
                heapq.heappush(heap, (n_cost + math.hypot(goal_col - n_col, goal_row - n_row),
                                      n_cost, (n_col, n_row)))
    return None


def line_free(blocked, a, b):
    '''True when the straight line between two cells crosses no blocked cell.'''
    count = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    cols = np.rint(np.linspace(a[0], b[0], count)).astype(int)
    rows = np.rint(np.linspace(a[1], b[1], count)).astype(int)
    return not blocked[rows, cols].any()


def shortcut(blocked, cells):
    '''Replace a staircase of cells by the fewest straight lines that stay clear.'''
    kept = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not line_free(blocked, cells[i], cells[j]):
            j -= 1
        kept.append(cells[j])
        i = j
    return kept


def resample(points, spacing):
    '''Points along a polyline at a fixed spacing, ending exactly on its last point.'''
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(int(math.ceil(length / spacing)), 1)
        for k in range(1, steps + 1):
            t = k / steps
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def plan_route(grid, start_xy, waypoints):
    '''
    Description:    Plan one rock-free path from the start through every waypoint in order.

    Args:
        grid (GridMap):     the rock layout
        start_xy (tuple):   where the rover is
        waypoints (list):   [(x, y), ...] in route order

    Returns:
        (points, waypoint_index, radius_m): the path as [(x, y), ...], the position of each
        waypoint inside it, and the body clearance that worked; or None.
        Clearance is relaxed step by step when the tightest setting finds no way through.
    '''

    stops = [start_xy] + list(waypoints)
    if not all(grid.inside(x, y) for x, y in stops):
        return None
    start_cell = grid.to_cell(*start_xy)
    multiplier = grid.cost_multiplier()

    for radius in planning_radii_m:
        blocked = grid.blocked_mask(radius, start_cell)
        goals = [grid.pass_cell(blocked, waypoint) for waypoint in waypoints]
        if any(goal is None for goal in goals):
            continue
        points = [start_xy]
        waypoint_index = []
        previous_cell, previous_xy = start_cell, start_xy
        for waypoint, goal in zip(waypoints, goals):
            found = astar(blocked, multiplier, previous_cell, goal)
            if found is None:
                break
            for k, (col, row) in enumerate(found):
                if math.hypot(grid.to_world(col, row)[0] - waypoint[0],
                              grid.to_world(col, row)[1] - waypoint[1]) <= trim_radius_m:
                    found = found[:k + 1]
                    break
            goal = found[-1]
            leg_cells = shortcut(blocked, found)
            goal_xy = grid.to_world(*goal)
            leg_points = [previous_xy] + [grid.to_world(c, r) for c, r in leg_cells[1:-1]] + [goal_xy]
            points.extend(resample(leg_points, path_spacing_m)[1:])
            waypoint_index.append(len(points) - 1)
            previous_cell, previous_xy = goal, goal_xy
        else:
            return points, waypoint_index, radius
    return None


def straight_route(start_xy, waypoints):
    '''Fallback when /map cannot be used: straight lines through the waypoints.'''
    points = [start_xy]
    waypoint_index = []
    previous = start_xy
    for waypoint in waypoints:
        points.extend(resample([previous, waypoint], path_spacing_m)[1:])
        waypoint_index.append(len(points) - 1)
        previous = waypoint
    return points, waypoint_index


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


class RouteFollower:
    '''
    ___CLASS___

    Description:    Follows a planned path with pure pursuit and keeps count of the waypoints.
                    It holds no ROS objects, so it can be tested on its own.
    '''

    def __init__(self, path, waypoint_index, waypoints, first_number=0):
        self.path = np.asarray(path, dtype=float)
        self.waypoint_index = waypoint_index
        self.waypoints = waypoints
        self.first_number = first_number          # route number of waypoints[0], for the log
        self.progress = 0
        self.target = 0
        self.reached_log = []
        self.reach = []                           # distance at which each waypoint counts as reached
        for number, (wx, wy) in enumerate(waypoints):
            end_x, end_y = self.path[waypoint_index[number]]
            route_gap = math.hypot(end_x - wx, end_y - wy)
            base = final_reach_m if number == len(waypoints) - 1 else reach_radius_m
            self.reach.append(min(max(base, route_gap + reach_slack_m), reach_cap_m))

    @property
    def done(self):
        return self.target >= len(self.waypoints)

    def distance_to_path(self, x, y):
        '''Move the progress marker forward and return how far the rover is from the path.'''
        low = self.progress
        high = min(len(self.path), low + progress_window)
        gaps = np.hypot(self.path[low:high, 0] - x, self.path[low:high, 1] - y)
        self.progress = low + int(np.argmin(gaps))
        return float(gaps.min())

    def update_waypoints(self, x, y):
        '''Count every waypoint the rover is now within reach of, strictly in order.'''
        while not self.done:
            wx, wy = self.waypoints[self.target]
            if math.hypot(wx - x, wy - y) > self.reach[self.target]:
                return
            self.progress = max(self.progress, self.waypoint_index[self.target])
            self.reached_log.append(self.first_number + self.target + 1)
            self.target += 1

    def aim_point(self, speed):
        '''The point on the path to steer at, never beyond the waypoint being chased.'''
        limit = self.waypoint_index[self.target]
        reach = lookahead_min_m + lookahead_gain_s * speed
        index, travelled = min(self.progress, limit), 0.0
        while index < limit and travelled < reach:
            travelled += float(np.hypot(*(self.path[index + 1] - self.path[index])))
            index += 1
        return self.path[index]

    def command(self, x, y, yaw, speed, obstacles):
        '''
        Description:    One control step.

        Args:
            x, y, yaw (float):      rover pose in the map frame
            speed (float):          measured forward speed (m/s)
            obstacles (ndarray):    lidar points in the body frame, shape (n, 2)

        Returns:
            (linear m/s, angular rad/s), inside the base's limits.
        '''

        if self.done:
            return 0.0, 0.0

        aim_x, aim_y = self.aim_point(speed)
        error = wrap_angle(math.atan2(aim_y - y, aim_x - x) - yaw)

        linear = cruise_mps * max(0.0, 1.0 - abs(error) / turn_in_place_rad)
        angular = heading_kp * error
        if abs(error) > heading_deadband_rad:
            angular = math.copysign(max(abs(angular), min_turn_rps), angular)
        else:
            angular = 0.0

        if self.target == len(self.waypoints) - 1:
            wx, wy = self.waypoints[-1]
            linear = min(linear, max(0.1, 0.5 * math.hypot(wx - x, wy - y)))
            if math.hypot(wx - x, wy - y) > final_slow_m:
                linear = min(linear, cruise_mps)

        linear, angular = self._avoid(linear, angular, obstacles, error)
        return (min(max(linear, -max_linear_mps), max_linear_mps),
                min(max(angular, -max_angular_rps), max_angular_rps))

    @staticmethod
    def _avoid(linear, angular, obstacles, heading_error):
        '''Slow down for anything in the corridor ahead of the body, and turn away if it is close.'''
        if len(obstacles) == 0:
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

        if abs(heading_error) > turn_toward_route_rad:
            return 0.0, angular                   # the planned route is clear: first face it, standing still
        side = float(np.median(ahead[:, 1]))
        return 0.0, -math.copysign(avoid_turn_rps, side if abs(side) > 1.0e-3 else 1.0)


##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the route
                    published on /ebot_path, through its waypoints, in order.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # The route and the map are published once and held for late subscribers, so the
        # subscriptions must ask for the same durability.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, latched)
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.mapcb, latched)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = Buffer()                                                       # where the lidar sits on the body
        self.tf_listener = TransformListener(self.tf_buffer, self)

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())

        ############ ADD YOUR CODE HERE ############

        self.grid = None                                                                # the rock layout (from mapcb())
        self.pose = None                                                                # (x, y, yaw) of the body
        self.speed = 0.0                                                                # measured forward speed
        self.lidar_mount = None                                                         # (x, y, yaw) of the lidar on the body
        self.follower = None                                                            # RouteFollower once a path is planned
        self.replans = 0
        self.reported_done = False
        self.failure_reported = False

        ############################################


    def pathcb(self, data):
        '''
        Description:    Callback function for the route topic.
                        Use this function to receive the waypoints the eBot has to drive.

        Args:
            data (Path):    The route, as a sequence of poses

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # The route is taken once: replacing it mid-run would reset the progress.
        if self.route is not None:
            return

        self.route = [(p.pose.position.x, p.pose.position.y) for p in data.poses]
        self.get_logger().info(
            f'Route received: {len(self.route)} waypoints in frame "{data.header.frame_id}"')

        ############################################


    def mapcb(self, data):
        '''
        Description:    Callback function for the occupancy grid of the rocks.

        Args:
            data (OccupancyGrid):    The rock layout

        Returns:
        '''

        if self.grid is not None:
            return

        self.grid = GridMap(data)
        self.get_logger().info(
            f'Map received: {self.grid.width}x{self.grid.height} cells at '
            f'{self.grid.resolution:.3f} m, {int(self.grid.occupied.sum())} rock cells, '
            f'{self.grid.unknown_cells} unknown cells')


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
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (position.x, position.y, yaw)
        self.speed = data.twist.twist.linear.x
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
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.lidar_mount = (t.x, t.y, yaw)
        self.get_logger().info(
            f'Lidar mount: x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f}')
        return True


    def plan(self, waypoints, first_number):
        '''
        Description:    Plan a rock-free path through the remaining waypoints and start following it.
        '''

        x, y, _ = self.pose
        planned = plan_route(self.grid, (x, y), waypoints)

        if planned is None:
            self.get_logger().error(
                'No rock-free path found on /map; driving straight between waypoints')
            points, waypoint_index = straight_route((x, y), waypoints)
        else:
            points, waypoint_index, radius = planned
            self.get_logger().info(
                f'Planned {len(points)} path points through waypoints '
                f'{first_number + 1}..{first_number + len(waypoints)} '
                f'(body clearance {radius:.2f} m)')

        self.follower = RouteFollower(points, waypoint_index, waypoints, first_number)


    def publish_velocity(self, linear, angular):
        '''Send one Twist; only linear.x and angular.z do anything on the eBot.'''
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)


    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

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
                self.get_logger().error(f'Navigation step failed, base stopped: {error!r}')

        ############################################


    def navigate(self):
        '''
        Description:    One control cycle: plan once, then steer along the path.
        '''

        if self.route is None or self.pose is None or self.scan is None or self.grid is None:
            return
        if not self.read_lidar_mount():
            return

        if self.follower is None:
            self.plan(self.route, 0)
            return

        x, y, yaw = self.pose
        follower = self.follower

        off_path = follower.distance_to_path(x, y)
        reached_before = len(follower.reached_log)
        follower.update_waypoints(x, y)
        for number in follower.reached_log[reached_before:]:
            self.get_logger().info(f'Waypoint {number}/{len(self.route)} reached')

        if follower.done:
            self.publish_velocity(0.0, 0.0)
            if not self.reported_done:
                self.reported_done = True
                self.get_logger().info('All waypoints reached. Base stopped.')
            return

        if off_path > replan_distance_m and self.replans < max_replans:
            self.replans += 1
            self.get_logger().warn(
                f'{off_path:.2f} m off the path; planning again ({self.replans}/{max_replans})')
            done_count = follower.first_number + follower.target
            self.plan(self.route[done_count:], done_count)
            return

        mount_x, mount_y, mount_yaw = self.lidar_mount
        obstacles = scan_to_base_points(self.scan, mount_x, mount_y, mount_yaw)
        linear, angular = follower.command(x, y, yaw, self.speed, obstacles)
        self.publish_velocity(linear, angular)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_nav_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot navigation process') # logging information

    ebot_nav_class = ebot_nav()                                     # creating a new object for class 'ebot_nav'

    rclpy.spin(ebot_nav_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_nav_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
