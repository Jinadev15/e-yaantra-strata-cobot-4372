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
# Filename:		    task3B_planning.py
# Functions:
#			        wrap_angle, yaw_from_quaternion, resample, cells_are_connected,
#			        polyline_length
# Classes:          OccupancyMap, CoarseGrid, AStarSearch, PathPlanner, StillnessTracker, ScanMemory,
#                   LapPhase, EbotState, ebot_planner
# Nodes:		    ebot_planner_node
#                   Publishing Topics  - [ /ebot_path, /handover/ebot ]
#                   Subscribing Topics - [ /map, /odom, /scan, /tf_static, /handover/arm ]
#                   Service Clients    - [ /spawn_ore_package ]


################### IMPORT MODULES #######################

import heapq
import math
import sys
from collections import deque, namedtuple
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


##################### TASK CONSTANTS #######################

# The map is published ONCE, as the world loads, and held for late subscribers.
map_topic = '/map'

# Your follower reads the path from here, with the same held-message QoS.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
spawn_service = '/spawn_ore_package'
body_frame = 'base_footprint'
lidar_frame = 'laser'

# The handover with the arm: latched string topics, so a late subscriber reads the state at once.
ebot_state_topic = '/handover/ebot'
arm_state_topic = '/handover/arm'
arm_package_clear = 'PACKAGE_CLEAR'           # the arm holds the package clear of the deck

# Every pose in the path, and the path itself, is in this frame.
map_frame = 'map'

# The three points of the lap: (x, y) in metres and yaw in radians, in the map frame.
# At the ore drop pose either yaw, 0.0 or 3.14, is valid. The arm pose is the stop beside the
# arm: free on every rock map, inside the arena wall (world x -0.47), and it puts the package
# on the deck about 0.67 m in front of the arm's base, inside its reach and in the camera's view.
home_pose = (0.0, 0.0, 0.0)
ore_drop_pose = (6.0, 1.9, 0.0)
arm_pose = (0.0, 1.9, 0.0)

# The lap, as (start, end) of each step, in the order they are driven.
steps = [
    (home_pose, ore_drop_pose),     # step 1: then request the ore package
    (ore_drop_pose, arm_pose),      # step 2
    (arm_pose, home_pose),          # step 3
]

# The yaws that count as correct at the end of each step, in radians. Home takes 0 only.
# At the ore drop pose the eBot turns to face west, the way step 2 leaves: it costs no time
# (step 2 would start with that turn), and its lidar then sees step 2's way before it is planned.
accepted_end_yaws = [
    (math.pi,),                     # ore drop pose
    (0.0, math.pi),                 # arm pose
    (0.0,),                         # home
]

# eBot footprint (eBot manual): 0.62 m long, 0.49 m wide.
half_width_m = 0.245

# --- Planning on /map ---
occupied_threshold = 50                       # occupancy value from which a cell is a rock
clearance_radii_m = (0.36, 0.34, 0.32, 0.30, 0.28, 0.26)   # body clearance tried in order, the widest that finds a way first: it keeps the body off the rocks, and past 0.36 m the detours cost more path than they are worth
pose_area_m = 0.30                            # the area around a step's start and end where the clearance is relaxed
spin_radius_m = 0.40                          # clearance kept at a pose so the eBot can turn on the spot there (reaches the footprint corners)
pose_radii_m = (spin_radius_m, half_width_m)  # tried in order: spin-safe first, the bare half-width only if a pose sits tight against a rock
search_weights = (0.0, 1.0, 2.0)              # extra path cost next to rocks: each weight can pick a different way round the rocks, and the shortest result wins
clearance_range_m = 1.0                       # the extra cost fades to zero this far from a rock
coarse_factor = 4                             # the search runs on cells this many times wider than the map's; the result is then pulled tight on the full map
tighten_passes = 60                           # most rounds of pulling the path tight
tighten_gain_m = 1.0e-4                       # stop when a round shortens the path by less than this
tighten_bisections = 14                       # how finely a corner is slid towards the straight line
path_spacing_m = 0.10                         # spacing of the points the eBot follows
arrival_lookback_points = 5                   # path points that give the heading the eBot arrives with

# --- Deciding that a step has finished (the task allows 0.3 m and 0.15 rad) ---
finish_position_m = 0.25                      # inside the allowed 0.3 m, with a margin
finish_yaw_rad = 0.10                         # inside the allowed 0.15 rad, with a margin
still_speed_mps = 0.02                        # below this the base is standing still
still_turn_rps = 0.05
still_window = 12                             # odometry twists averaged (about 0.3 s): single readings carry noise of 0.02 m/s and 0.04 rad/s
still_hold_s = 1.0                            # stand still this long before a step counts as finished
start_hold_s = 0.5                            # stand still this long before the next path is planned
start_radius_m = 1.0                          # a path is planned only when the eBot is this close to the step's start

# --- Rocks the map does not show (the map may not match the world) ---
# Before a step is planned, lidar returns that no mapped rock explains are added to the
# planning map as rocks. The published path then still crosses no occupied /map cell.
scan_window = 5                               # scans taken standing still; the per-beam median removes sensor noise
unmapped_min_distance_m = 0.15                # a return this far from every mapped rock is something /map does not show
unmapped_max_range_m = 4.0                    # farther returns are left to the next step's look
unmapped_support = 3                          # a return counts only with this many others close by: a lone return is noise
unmapped_support_radius_m = 0.10
unmapped_mark_radius_m = 0.05                 # each counted return blocks a disc this wide on the planning map
scan_wait_s = 3.0                             # plan on the map alone if no full window of still scans arrives in this time

# --- Ore package request ---
request_retry_s = 1.0                         # wait between requests when the service refuses
request_max_tries = 30
settle_after_package_s = 2.0                  # let the package come to rest on the deck before driving off


##################### HELPER FUNCTIONS #######################

Pose = namedtuple('Pose', 'x y yaw')


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


def resample(points, spacing):
    '''
    Description:    Points along a polyline at an even spacing, ending exactly on its last point.

    Args:
        points (list):      [(x, y), ...], at least two
        spacing (float):    wanted distance between neighbours, in metres

    Returns:
        list:               [(x, y), ...] starting and ending exactly on the input's ends
    '''

    out = [points[0]]
    for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
        count = max(math.ceil(math.hypot(bx - ax, by - ay) / spacing), 1)
        out.extend((ax + (bx - ax) * k / count, ay + (by - ay) * k / count)
                   for k in range(1, count))
        out.append((bx, by))                                      # exact, not accumulated rounding
    return out


def cells_are_connected(blocked, a, b):
    '''
    Description:    True when the straight line between two (column, row) cells crosses no
                    blocked cell.
    '''

    count = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
    cols = np.rint(np.linspace(a[0], b[0], count)).astype(int)
    rows = np.rint(np.linspace(a[1], b[1], count)).astype(int)
    return not blocked[rows, cols].any()


def polyline_length(points):
    '''
    Description:    Length in metres of the polyline through 'points'.
    '''

    return sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))


##################### CLASS DEFINITION #######################

class OccupancyMap:
    '''
    ___CLASS___

    Description:    The /map occupancy grid. Converts between world and cell, knows the
                    distance from every cell to the nearest rock, and grows the rocks into
                    the set of cells the eBot centre may not enter.
    '''

    def __init__(self, info, data):
        '''
        Args:
            info (MapMetaData):     resolution, size and origin of the grid
            data (ndarray):         occupancy values, shape (height, width)
        '''
        self.info = info
        self.resolution = info.resolution
        self.width = info.width
        self.height = info.height
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y

        self.data = data
        self.rock_cells = int(np.count_nonzero(data >= occupied_threshold))
        self.unknown_cells = int(np.count_nonzero(data < 0))

        if self.rock_cells:
            self.rock_distance_m = distance_transform_edt(data < occupied_threshold) * self.resolution
        else:
            self.rock_distance_m = np.full(data.shape, 1.0e3)

    @classmethod
    def from_message(cls, msg):
        '''The map as published on /map.'''
        return cls(msg.info, np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width))

    def with_obstacles(self, points, radius_m):
        '''A copy of this map with a rock disc of radius_m round each world point (x, y).'''
        data = self.data.copy()
        reach = math.ceil(radius_m / self.resolution)
        d_row, d_col = np.mgrid[-reach:reach + 1, -reach:reach + 1]
        disc = np.hypot(d_row, d_col) <= reach
        cells = np.array([self.to_cell(x, y) for x, y in points])
        rows = np.clip(cells[:, 1:2] + d_row[disc], 0, self.height - 1)
        cols = np.clip(cells[:, 0:1] + d_col[disc], 0, self.width - 1)
        data[rows.ravel(), cols.ravel()] = 100
        return OccupancyMap(self.info, data)

    def rock_distance_at(self, points):
        '''Distance in metres from each world point (x, y) to the nearest rock; inf off the grid.'''
        cols = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)
        inside = (cols >= 0) & (rows >= 0) & (cols < self.width) & (rows < self.height)
        distance = np.full(len(points), np.inf)
        distance[inside] = self.rock_distance_m[rows[inside], cols[inside]]
        return distance

    def to_cell(self, x, y):
        '''World (x, y) to (column, row), clamped to the grid.'''
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return min(max(col, 0), self.width - 1), min(max(row, 0), self.height - 1)

    def to_world(self, col, row):
        '''Centre of a (column, row) cell in world coordinates.'''
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def contains(self, x, y):
        '''True when the world point lies on the grid.'''
        col = (x - self.origin_x) / self.resolution
        row = (y - self.origin_y) / self.resolution
        return 0.0 <= col < self.width and 0.0 <= row < self.height

    def blocked_cells(self, radius_m, pose_cells, pose_radius_m):
        '''
        Cells the eBot centre may not enter: those within radius_m of a rock.
        Around each cell in 'pose_cells' (a step's start and end) the clearance is relaxed to
        pose_radius_m, so the eBot can still leave and reach a pose that sits beside a rock.
        '''
        blocked = self.rock_distance_m <= radius_m
        half = math.ceil(pose_area_m / self.resolution)
        near_pose = min(radius_m, pose_radius_m)
        for col, row in pose_cells:
            rows = slice(max(row - half, 0), row + half + 1)
            cols = slice(max(col - half, 0), col + half + 1)
            blocked[rows, cols] = self.rock_distance_m[rows, cols] <= near_pose
        return blocked

    def segment_is_free(self, blocked, a, b):
        '''True when the straight line between two world points crosses no blocked cell.'''
        count = int(math.dist(a, b) / (0.5 * self.resolution)) + 2
        cols = np.floor((np.linspace(a[0], b[0], count) - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((np.linspace(a[1], b[1], count) - self.origin_y) / self.resolution).astype(int)
        if cols.min() < 0 or rows.min() < 0 or cols.max() >= self.width or rows.max() >= self.height:
            return False
        return not blocked[rows, cols].any()


class CoarseGrid:
    '''
    ___CLASS___

    Description:    The map seen through cells 'factor' times wider. The search for a way round
                    the rocks runs here, where it is factor-squared times cheaper; a coarse cell
                    is blocked when any cell inside it is, so what the search finds is always
                    clear on the full map too.
    '''

    def __init__(self, fine_map, factor):
        self.fine = fine_map
        self.factor = factor
        self.rows = -(-fine_map.height // factor)
        self.cols = -(-fine_map.width // factor)

        padded = np.full((self.rows * factor, self.cols * factor), 1.0e3)
        padded[:fine_map.height, :fine_map.width] = fine_map.rock_distance_m
        self.rock_distance_m = padded.reshape(self.rows, factor, self.cols, factor).min(axis=(1, 3))

    def block(self, fine_blocked):
        '''The coarse cells that hold at least one blocked cell (the padding counts as blocked).'''
        fine_map, factor = self.fine, self.factor
        padded = np.ones((self.rows * factor, self.cols * factor), dtype=bool)
        padded[:fine_map.height, :fine_map.width] = fine_blocked
        return padded.reshape(self.rows, factor, self.cols, factor).any(axis=(1, 3))

    def step_costs(self, weight):
        '''Per-cell path cost factor as a flat list: 1 far from rocks, rising towards them.'''
        closeness = np.clip(1.0 - self.rock_distance_m / clearance_range_m, 0.0, 1.0)
        return (1.0 + weight * closeness).ravel().tolist()

    def cell_of(self, fine_cell):
        '''The coarse (column, row) that holds a full-map (column, row).'''
        return fine_cell[0] // self.factor, fine_cell[1] // self.factor

    def to_world(self, col, row):
        '''Centre of a coarse (column, row) cell in world coordinates.'''
        fine_map, factor = self.fine, self.factor
        return (fine_map.origin_x + (col + 0.5) * factor * fine_map.resolution,
                fine_map.origin_y + (row + 0.5) * factor * fine_map.resolution)


class AStarSearch:
    '''
    ___CLASS___

    Description:    A* on a grid, 8-connected, that never cuts a blocked corner. Cells are
                    numbered row * width + col, so the search works on flat lists, which is
                    several times faster than dictionaries of (column, row) pairs.
    '''

    def __init__(self, width, height, step_cost):
        self.width = width
        self.height = height
        self.step_cost = step_cost
        diagonal = math.sqrt(2.0)
        self.moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                      (1, 1, diagonal), (1, -1, diagonal), (-1, 1, diagonal), (-1, -1, diagonal))

    def search(self, blocked, start, goal):
        '''
        Args:
            blocked (ndarray):    True where the eBot centre may not go, shape (height, width)
            start, goal (tuple):  (column, row)

        Returns:
            list of (column, row) from start to goal, or None when there is no way through.
        '''

        width, height = self.width, self.height
        flat_blocked = blocked.ravel().tolist()
        goal_col, goal_row = goal
        start_id, goal_id = start[1] * width + start[0], goal_row * width + goal_col

        best = [math.inf] * (width * height)
        parent = [-1] * (width * height)
        best[start_id] = 0.0
        heap = [(math.hypot(goal_col - start[0], goal_row - start[1]), start_id)]

        while heap:
            _, cell = heapq.heappop(heap)
            if cell == goal_id:
                return self._cells_to(cell, parent)
            row, col = divmod(cell, width)
            cost = best[cell]
            for d_col, d_row, length in self.moves:
                n_col, n_row = col + d_col, row + d_row
                if not (0 <= n_col < width and 0 <= n_row < height):
                    continue
                neighbour = n_row * width + n_col
                if flat_blocked[neighbour]:
                    continue
                if d_col and d_row and (flat_blocked[row * width + n_col]
                                        or flat_blocked[n_row * width + col]):
                    continue
                n_cost = cost + length * self.step_cost[neighbour]
                if n_cost < best[neighbour]:
                    best[neighbour] = n_cost
                    parent[neighbour] = cell
                    heapq.heappush(heap, (n_cost + math.hypot(goal_col - n_col, goal_row - n_row),
                                          neighbour))
        return None

    def _cells_to(self, cell, parent):
        '''Walk the parent links back from 'cell' and return the cells start-first.'''
        cells = []
        while cell != -1:
            cells.append(divmod(cell, self.width)[::-1])
            cell = parent[cell]
        return cells[::-1]


class PathPlanner:
    '''
    ___CLASS___

    Description:    Plans a rock-free path between two poses on the map. The rocks are grown by
                    the body clearance and A* finds a way round them on a coarse grid; every
                    cost weight may choose a different way, and each way is then pulled tight on
                    the full map like a string, so that it only bends where a rock holds it.
                    The shortest result is the path.
    '''

    def __init__(self, occupancy_map):
        self.map = occupancy_map
        self.coarse = CoarseGrid(occupancy_map, coarse_factor)
        self.searches = [AStarSearch(self.coarse.cols, self.coarse.rows, self.coarse.step_costs(weight))
                         for weight in search_weights]
        self.fine_search = None                   # built only if the coarse grid ever finds no way through

    def plan(self, start_xy, end_xy):
        '''
        Description:    Plan a path from start_xy to end_xy.

        Args:
            start_xy, end_xy (tuple):   (x, y) in the map frame

        Returns:
            (points, clearance_m): the path as [(x, y), ...] that begins and ends exactly on
            start_xy and end_xy, and the body clearance that worked; or None.
            The clearance is relaxed step by step when the widest setting finds no way through.
        '''

        grid = self.map
        if not (grid.contains(*start_xy) and grid.contains(*end_xy)):
            return None
        start, goal = grid.to_cell(*start_xy), grid.to_cell(*end_xy)

        for pose_radius in pose_radii_m:
            for radius in clearance_radii_m:
                blocked = grid.blocked_cells(radius, (start, goal), pose_radius)
                if blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
                    continue
                paths = []
                for corners in self._corner_sets(blocked, start, goal):
                    points = self._tighten(blocked, [tuple(start_xy)] + corners + [tuple(end_xy)])
                    if self._is_free(blocked, points):
                        paths.append(points)
                if paths:
                    return resample(min(paths, key=polyline_length), path_spacing_m), radius
        return None

    def _corner_sets(self, blocked, start, goal):
        '''
        Every way round the rocks that the search finds, one per cost weight, each as the
        world points of its corners. Falls back to the full-size map when the coarse one
        has no way through.
        '''

        coarse = self.coarse
        coarse_blocked = coarse.block(blocked)
        coarse_start, coarse_goal = coarse.cell_of(start), coarse.cell_of(goal)
        coarse_blocked[coarse_start[1], coarse_start[0]] = False
        coarse_blocked[coarse_goal[1], coarse_goal[0]] = False

        corner_sets = []
        for search in self.searches:
            cells = search.search(coarse_blocked, coarse_start, coarse_goal)
            if cells is None:
                break
            corners = self._straighten(coarse_blocked, cells)
            corner_sets.append([coarse.to_world(col, row) for col, row in corners[1:-1]])
        if corner_sets:
            return corner_sets

        if self.fine_search is None:
            self.fine_search = AStarSearch(self.map.width, self.map.height,
                                           [1.0] * (self.map.width * self.map.height))
        cells = self.fine_search.search(blocked, start, goal)
        if cells is None:
            return []
        corners = self._straighten(blocked, cells)
        return [[self.map.to_world(col, row) for col, row in corners[1:-1]]]

    @staticmethod
    def _straighten(blocked, cells):
        '''Replace a staircase of cells by the fewest straight lines that stay clear.'''
        kept = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not cells_are_connected(blocked, cells[i], cells[j]):
                j -= 1
            kept.append(cells[j])
            i = j
        return kept

    def _tighten(self, blocked, points):
        '''
        Pull the path tight like a string: drop a corner when its neighbours can see each
        other, otherwise slide it as far towards their straight line as the rocks allow.
        '''

        grid = self.map
        points = list(points)
        for _ in range(tighten_passes):
            before = polyline_length(points)
            i = 1
            while i < len(points) - 1:
                a, corner, b = points[i - 1], points[i], points[i + 1]
                if grid.segment_is_free(blocked, a, b):
                    del points[i]
                    continue
                middle = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
                low, high = 0.0, 1.0
                for _ in range(tighten_bisections):
                    t = (low + high) / 2.0
                    moved = (corner[0] + (middle[0] - corner[0]) * t,
                             corner[1] + (middle[1] - corner[1]) * t)
                    if grid.segment_is_free(blocked, a, moved) and grid.segment_is_free(blocked, moved, b):
                        low = t
                    else:
                        high = t
                points[i] = (corner[0] + (middle[0] - corner[0]) * low,
                             corner[1] + (middle[1] - corner[1]) * low)
                i += 1
            if before - polyline_length(points) < tighten_gain_m:
                break
        return points

    def _is_free(self, blocked, points):
        '''True when every straight piece of the path crosses no blocked cell.'''
        return all(self.map.segment_is_free(blocked, a, b) for a, b in zip(points[:-1], points[1:]))


class StillnessTracker:
    '''
    ___CLASS___

    Description:    Tells how long the base has been standing still, from its odometry twist.
    '''

    def __init__(self):
        self.since = None                         # simulation time (s) the base stopped, or None while moving
        self.twists = deque(maxlen=still_window)  # recent (forward speed, turn rate)

    def update(self, twist, now_s):
        '''Feed one odometry twist and the current simulation time in seconds.'''
        self.twists.append((twist.linear.x, twist.angular.z))
        speed, turn = np.mean(self.twists, axis=0)
        moving = abs(speed) > still_speed_mps or abs(turn) > still_turn_rps
        if moving:
            self.since = None
        elif self.since is None:
            self.since = now_s

    def still_for(self, now_s):
        '''Seconds the base has stood still, 0.0 while it is moving.'''
        return 0.0 if self.since is None else now_s - self.since


class ScanMemory:
    '''
    ___CLASS___

    Description:    The last few lidar scans taken while the eBot stood still. The median of
                    each beam over them is a steady picture of the surroundings; from it the
                    returns that no mapped rock explains are picked out.
    '''

    def __init__(self):
        self.ranges = deque(maxlen=scan_window)
        self.scan = None                          # the latest scan, for its beam geometry

    @property
    def full(self):
        return len(self.ranges) == scan_window

    def add(self, scan):
        ranges = np.asarray(scan.ranges, dtype=float)
        if self.ranges and len(self.ranges[-1]) != len(ranges):
            self.ranges.clear()
        self.ranges.append(ranges)
        self.scan = scan

    def clear(self):
        self.ranges.clear()

    def unmapped_points(self, grid, pose, mount):
        '''
        Args:
            grid (OccupancyMap):    the map to explain the returns with
            pose (Pose):            the body in the map frame
            mount (tuple):          (x, y, yaw) of the lidar on the body

        Returns:
            ndarray (n, 2) of map-frame points the map does not explain, noise removed.
        '''

        scan = self.scan
        stacked = np.vstack(self.ranges)
        stacked[~np.isfinite(stacked)] = np.nan
        ranges = np.nanmedian(stacked, axis=0) if np.isfinite(stacked).any() else np.full(stacked.shape[1], np.nan)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= min(scan.range_max, unmapped_max_range_m))
        if not valid.any():
            return np.empty((0, 2))
        ranges, angles = ranges[valid], angles[valid] + mount[2]
        body = np.column_stack((mount[0] + ranges * np.cos(angles), mount[1] + ranges * np.sin(angles)))
        cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
        world = np.column_stack((pose.x + cos_yaw * body[:, 0] - sin_yaw * body[:, 1],
                                 pose.y + sin_yaw * body[:, 0] + cos_yaw * body[:, 1]))
        distance = grid.rock_distance_at(world)
        unexplained = world[np.isfinite(distance) & (distance > unmapped_min_distance_m)]
        if len(unexplained) <= unmapped_support:
            return np.empty((0, 2))
        neighbours = cKDTree(unexplained).query_ball_point(unexplained, unmapped_support_radius_m, return_length=True)
        return unexplained[neighbours > unmapped_support]


class LapPhase(Enum):
    '''Where the planner is within one step of the lap.'''

    PLAN = 1              # wait for the eBot to stand still at the start, then plan and publish
    DRIVE = 2             # the follower is driving; wait for the eBot to stop on the end pose
    PACKAGE = 3           # step 1 only: request the ore package
    SETTLE = 4            # step 1 only: let the package come to rest
    HANDOVER = 5          # step 2 only: stand still beside the arm until the package is off the deck
    FINISHED = 6


class EbotState(Enum):
    '''What this node reports on /handover/ebot.'''

    EN_ROUTE = 'EN_ROUTE'
    AT_ARM = 'AT_ARM'                             # standing still beside the arm, package on the deck
    LEAVING = 'LEAVING'


class ebot_planner(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to read the map, plan a path for each
                    step of the lap, publish it on /ebot_path, request the ore package at the
                    end of step 1, and hold the eBot beside the arm after step 2 until the arm
                    reports the package lifted off the deck.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_planner
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_planner_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # A held message: kept by the publisher and handed to anyone who subscribes later.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        ############ Topic SUBSCRIPTIONS ############

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.mapcb, latched)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.path_pub = self.create_publisher(Path, path_topic, latched)                # your follower drives what you publish here
        self.state_pub = self.create_publisher(String, ebot_state_topic, latched)       # handover state for the arm
        self.arm_state_sub = self.create_subscription(String, arm_state_topic, self.armstatecb, latched)

        ############ Service CLIENTS ############

        self.spawn_client = self.create_client(Trigger, spawn_service)                  # drops the ore package onto the eBot

        ############ Constructor VARIABLES/OBJECTS ############

        plan_rate = 0.1                                                                 # rate of time to run one planning cycle (seconds)
        self.timer = self.create_timer(plan_rate, self.process_planning)                # creating a timer based function which gets called on every 0.1 seconds (as defined by 'plan_rate' variable)

        self.grid = None                                                                # the map (from mapcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.step = 0                                                                   # which step of the lap is next (index into 'steps')

        ############ ADD YOUR CODE HERE ############

        self.planner = None                                                             # PathPlanner, built when the map arrives
        self.pose = None                                                                # Pose of the base (from odomcb())
        self.stillness = StillnessTracker()                                             # how long the base has stood still
        self.phase = LapPhase.PLAN
        self.end_yaw = 0.0                                                              # the yaw the current step's path ends on
        self.package_tries = 0
        self.package_in_flight = False                                                  # a request is waiting for its reply
        self.package_granted = False
        self.last_request_s = None
        self.settle_started_s = None
        self.arm_state = ''                                                             # last handover state of the arm
        self.scans = ScanMemory()                                                       # still scans for spotting unmapped rocks
        self.tf_buffer = Buffer()                                                       # where the lidar sits on the body
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_mount = None
        self.plan_wait_since_s = None                                                   # when the eBot became ready to plan this step
        self.publish_state(EbotState.EN_ROUTE)

        ############################################


    def mapcb(self, data):
        '''
        Description:    Callback function for the map topic.
                        Use this function to receive the occupancy grid of the arena.

        Args:
            data (OccupancyGrid):    The arena, one value per cell

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if self.grid is not None:                                                       # published once; the first copy is the map
            return

        self.grid = OccupancyMap.from_message(data)
        self.planner = PathPlanner(self.grid)
        self.get_logger().info(
            f'Map received: {self.grid.width}x{self.grid.height} cells at '
            f'{self.grid.resolution:.3f} m, origin ({self.grid.origin_x:.2f}, '
            f'{self.grid.origin_y:.2f}), {self.grid.rock_cells} rock cells, '
            f'{self.grid.unknown_cells} unknown cells')

        ############################################


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic. Only scans taken while the
                        eBot stands still are kept: they all see the same surroundings.

        Args:
            data (LaserScan):    One lidar sweep
        '''

        if self.stillness.still_for(self.now_s()) > 0.0:
            self.scans.add(data)
        else:
            self.scans.clear()


    def read_lidar_mount(self):
        '''Where the lidar sits on the body, from /tf_static. Returns True once known.'''
        if self.lidar_mount is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(body_frame, lidar_frame, Time())
        except Exception:
            return False
        t = tf.transform.translation
        self.lidar_mount = (t.x, t.y, yaw_from_quaternion(tf.transform.rotation))
        return True


    def armstatecb(self, data):
        '''
        Description:    Callback function for the arm's handover state.

        Args:
            data (String):    State published by the manipulation node
        '''

        self.arm_state = data.data


    def publish_state(self, state):
        '''Publish the eBot's handover state; the topic is latched.'''
        self.state_pub.publish(String(data=state.value))
        self.get_logger().info(f'Handover: eBot {state.value}')


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
        self.pose = Pose(position.x, position.y, yaw_from_quaternion(data.pose.pose.orientation))
        self.odom = data
        self.stillness.update(data.twist.twist, self.now_s())

        ############################################


    def step_planner(self):
        '''
        The planner for the current step: on /map with every unmapped return the lidar sees
        from here added as a rock, or on /map alone when nothing unmapped is in view.
        '''

        if not (self.scans.full and self.read_lidar_mount()):
            return self.planner
        points = self.scans.unmapped_points(self.grid, self.pose, self.lidar_mount)
        if len(points) == 0:
            return self.planner
        self.get_logger().info(f'Step {self.step + 1}: {len(points)} lidar returns not on the map, '
                               f'planning round them')
        return PathPlanner(self.grid.with_obstacles(points, unmapped_mark_radius_m))


    def plan_path(self, start, end):
        '''
        Description:    Plans a path from 'start' to 'end' that stays clear of every rock.

        Args:
            start (tuple):    (x, y, yaw) where the step begins, in the map frame
            end (tuple):      (x, y, yaw) where the step ends, in the map frame

        Returns:
            list:             [(x, y), ...] from 'start' to 'end', or None if no path was found
        '''

        points = None

        ############ ADD YOUR CODE HERE ############

        planner = self.step_planner()
        planned = planner.plan(start[:2], end[:2])
        if planned is None and planner is not self.planner:
            self.get_logger().warn(f'Step {self.step + 1}: no way round the unmapped returns; planning on /map alone')
            planned = self.planner.plan(start[:2], end[:2])
        if planned is not None:
            points, clearance = planned
            length = sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))
            self.get_logger().info(
                f'Step {self.step + 1}: planned {len(points)} points, {length:.2f} m, '
                f'body clearance {clearance:.2f} m')

        ############################################

        return points


    def publish_path(self, points, start, end):
        '''
        Description:    Publishes 'points' on /ebot_path as a nav_msgs/Path.

        Args:
            points (list):    [(x, y), ...] as returned by plan_path()
            start (tuple):    (x, y, yaw) where the step begins
            end (tuple):      (x, y, yaw) where the step ends

        Returns:
        '''

        path = Path()
        path.header.frame_id = map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        ############ ADD YOUR CODE HERE ############

        last = len(points) - 1
        for index, (x, y) in enumerate(points):
            if index == 0:
                yaw = start[2]
            elif index == last:
                yaw = end[2]
            else:
                yaw = math.atan2(points[index + 1][1] - y, points[index + 1][0] - x)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)

        self.path_pub.publish(path)

        ############################################


    def request_ore_package(self):
        '''
        Description:    Calls /spawn_ore_package once the eBot stands still on the ore drop pose.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if not self.spawn_client.service_is_ready():
            return

        self.package_tries += 1
        self.package_in_flight = True
        self.last_request_s = self.now_s()
        self.spawn_client.call_async(Trigger.Request()).add_done_callback(self.package_replied)

        ############################################


    def package_replied(self, future):
        '''
        Description:    Handles the reply of /spawn_ore_package. A refusal is tried again
                        shortly, since the base may still have been settling.
        '''

        self.package_in_flight = False
        try:
            reply = future.result()
        except Exception as error:                                                      # the service went away mid-call
            self.get_logger().warn(f'Ore package request failed: {error!r}')
            return

        self.get_logger().info(
            f'Ore package request {self.package_tries}: success={reply.success} '
            f'message="{reply.message}"')
        self.package_granted = reply.success


    def now_s(self):
        '''The simulation time in seconds.'''
        return self.get_clock().now().nanoseconds / 1.0e9


    def process_planning(self):
        '''
        Description:    Timer function used to plan, publish and request, step by step.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        if self.grid is None or self.pose is None or self.phase is LapPhase.FINISHED:
            return

        try:
            handler = {LapPhase.PLAN: self.plan_phase,
                       LapPhase.DRIVE: self.drive_phase,
                       LapPhase.PACKAGE: self.package_phase,
                       LapPhase.SETTLE: self.settle_phase,
                       LapPhase.HANDOVER: self.handover_phase}[self.phase]
            handler()
        except Exception as error:                                                      # an uncaught exception would kill the node
            self.get_logger().error(f'Planning cycle failed: {error!r}')

        ############################################


    def plan_phase(self):
        '''Plan and publish the current step once the eBot stands still near its start.'''

        start, end = steps[self.step]
        if self.stillness.still_for(self.now_s()) < start_hold_s:
            self.plan_wait_since_s = None
            return
        if math.hypot(self.pose.x - start[0], self.pose.y - start[1]) > start_radius_m:
            return
        if self.plan_wait_since_s is None:
            self.plan_wait_since_s = self.now_s()
        if not self.scans.full and self.now_s() - self.plan_wait_since_s < scan_wait_s:
            return                                                                      # a full window of still scans first

        points = self.plan_path(start, end)
        if points is None:
            self.get_logger().error(f'Step {self.step + 1}: no rock-free path found, trying again')
            return

        self.end_yaw = self.arrival_yaw(points, accepted_end_yaws[self.step])
        self.publish_path(points, start, (end[0], end[1], self.end_yaw))
        self.phase = LapPhase.DRIVE
        self.get_logger().info(f'Step {self.step + 1}: path published, ending at yaw {self.end_yaw:.2f}')


    @staticmethod
    def arrival_yaw(points, accepted):
        '''The accepted end yaw that needs the least turning after driving the path's last stretch.'''
        a = points[max(len(points) - 1 - arrival_lookback_points, 0)]
        b = points[-1]
        arrival = math.atan2(b[1] - a[1], b[0] - a[0])
        return min(accepted, key=lambda yaw: abs(wrap_angle(yaw - arrival)))


    def drive_phase(self):
        '''Wait for the eBot to stop on the step's end pose, then move on.'''

        end = steps[self.step][1]
        on_pose = (math.hypot(self.pose.x - end[0], self.pose.y - end[1]) <= finish_position_m
                   and abs(wrap_angle(self.pose.yaw - self.end_yaw)) <= finish_yaw_rad)
        if not on_pose or self.stillness.still_for(self.now_s()) < still_hold_s:
            return

        self.get_logger().info(f'Step {self.step + 1} finished')
        if self.step == 0:
            self.phase = LapPhase.PACKAGE
        elif self.step == 1:
            self.publish_state(EbotState.AT_ARM)
            self.phase = LapPhase.HANDOVER
        else:
            self.next_step()


    def handover_phase(self):
        '''Stand still beside the arm; leave only once the package is off the deck.'''

        if self.arm_state != arm_package_clear:
            return
        self.publish_state(EbotState.LEAVING)
        self.next_step()


    def package_phase(self):
        '''Ask for the ore package until it is granted, then let it settle.'''

        if self.package_granted or (self.package_tries >= request_max_tries and not self.package_in_flight):
            if not self.package_granted:
                self.get_logger().error('The ore package was never granted; carrying on with the lap')
            self.settle_started_s = self.now_s()
            self.phase = LapPhase.SETTLE
            return

        waited = math.inf if self.last_request_s is None else self.now_s() - self.last_request_s
        if not self.package_in_flight and waited >= request_retry_s:
            self.request_ore_package()


    def settle_phase(self):
        '''Give the package time to come to rest on the deck, then start step 2.'''

        if self.now_s() - self.settle_started_s >= settle_after_package_s:
            self.next_step()


    def next_step(self):
        '''Move on to the next step of the lap, or stop after step 3.'''

        self.step += 1
        if self.step >= len(steps):
            self.phase = LapPhase.FINISHED
            self.get_logger().info('Lap complete: home, ore drop pose, arm pose, home')
        else:
            self.phase = LapPhase.PLAN


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_planner class to perform its task
    '''

    rclpy.init(args=sys.argv)                                           # initialisation

    node = rclpy.create_node('ebot_planner_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot planner process')        # logging information

    ebot_planner_class = ebot_planner()                                 # creating a new object for class 'ebot_planner'

    rclpy.spin(ebot_planner_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_planner_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                    # shutdown process


if __name__ == '__main__':

    main()
