#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 3A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task3A_manipulation.py
# Functions:
#			        arc_points, clearance_height, main
# Classes:          ArmFeedback, Waypoint, CartesianServo, RouteFollower, JointUnfolder,
#                   ContainerMap, ItemLocator, ServiceCaller, EbotState, ArmState, Phase,
#                   ore_manipulation
# Nodes:		    ore_manipulation_node
#                   Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds, /handover/arm ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status, /handover/ebot, /odom,
#                                          /ur7e/end_effector/force_magnitude, /tf, /tf_static ]


################### IMPORT MODULES #######################

import math
import sys
from collections import deque
from dataclasses import dataclass
from enum import Enum

import numpy as np
import rclpy
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener


##################### TASK CONSTANTS #######################

base_frame = 'base_link'
world_frame = 'world'

servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts (Task 3: 0.25 m/s, 0.60 rad/s, 0.60 rad/s per joint): a command
# above one of these is dropped whole, not clamped, so each cap keeps a margin below it.
cap_linear_mps = 0.22
cap_angular_rps = 0.50
cap_joint_rps = 0.50

# The handover with the eBot: latched string topics, so a late subscriber reads the state at once.
ebot_state_topic = '/handover/ebot'
arm_state_topic = '/handover/arm'

# Arm status codes (The UR7e Arm): 0 is healthy, 10-14 warn, 20-23 are latched protective stops.
status_protective_stop_min = 20

ore_kinds = ('azurite', 'malachite', 'vanadinite')
ore_slots_per_kind = 4                         # an ore of one kind is numbered 1 up to this
item_height_m = 0.0762                         # ore and package are both this tall; a transform names the middle
expected_ore_count = 4
package_frame = 'ore_package'

# --- Start-up: leave the folded pose in joint space (the elbow starts 20 degrees from its limit) ---
unfold_target = {
    'shoulder_lift_joint': -1.20,
    'elbow_joint': -1.60,
    'wrist_1_joint': -1.90,
}
unfold_kp = 0.8                                # 1/s, joint error -> velocity
unfold_tolerance_rad = 0.02

# --- Cartesian motion ---
position_kp = 1.5                              # 1/s, position error -> tool speed
orientation_kp = 1.0                           # 1/s, orientation error -> angular speed
transit_speed_mps = 0.21
approach_speed_mps = 0.15
descent_speed_mps = 0.025                      # the arm coasts about 5 mm at 0.02 m/s and 50 mm at 0.10 m/s
lift_speed_mps = 0.06
travel_tolerance_m = 0.02
pass_tolerance_m = 0.05                        # a pass-through waypoint hands over this close, at full speed
precise_tolerance_m = 0.008
settle_s = 0.4                                 # stand still this long on arriving at a precise waypoint

# --- Heights, in base_link ---
hover_clearance_m = 0.12                       # tool face above an ore's top face before the descent
travel_height_m = 0.14                         # tool height while carrying: clears the bay rim by 0.1 m with the ore hanging below
swing_radius_m = 0.50                          # radius of the arc round the front of the base
swing_step_deg = 30.0
# The eBot stops in front of the base (azimuth 0) with its deck at z +0.18 and the package top
# at +0.26. Inside this sector the tool always travels high enough for a carried ore to pass
# over both, whether or not the eBot is there yet.
front_sector_rad = math.radians(50.0)
front_clear_height_m = 0.40
# The eBot counts as near the arm (map frame) inside this box: its last stretch towards the stop
# pose, the stop pose, and its way out. Unknown odometry counts as near.
ebot_near_max_x_m = 2.5
ebot_near_min_y_m = 1.0

# --- Pick ---
touch_force_n = 0.8                            # firm contact; the 0.4 N seen higher up is not the ore
touch_zone_m = 0.02                            # a touch only counts this close above the estimated top face
overpress_force_n = 3.0                        # pressing harder than this: stop and back off
contact_start_m = 0.02                         # the slow final stage starts this far above the estimated top face
contact_speed_mps = 0.01                       # speed of the final stage
search_depth_m = 0.005                         # the final stage aims this far below the estimated top face: the tool
                                               # spring takes up the difference, and the face lies flat on the ore
contact_tolerance_m = 0.003
grab_wait_s = 0.8                              # coil on and touching: wait before lifting
holding_force_n = (0.9, 2.2)                   # an ore (0.15 kg) adds about 1.47 N; free air reads about 0
package_holding_force_n = (2.3, 3.7)           # the package (0.30 kg) adds about 2.94 N
force_window = 10                              # samples averaged to judge the hold
contact_window = 3                             # samples averaged to judge touch and overpress: one noisy sample is not contact
max_pick_attempts = 3
descent_timeout_s = 25.0

# --- Drop: the bay centres in the world frame (from the rover's collision mesh) ---
# Bays run along world x, azurite nearest the arm, vanadinite farthest; the drop point is
# pulled towards the arm side of the bay for the far bays, which are at the edge of reach.
drop_points_world = {
    'azurite': (-0.626, 2.550),
    'malachite': (-0.932, 2.500),
    'vanadinite': (-1.224, 2.430),
}
drop_slot_offsets_m = (0.0, 0.065, -0.065, 0.0)  # along world x: the 2nd, 3rd ... ore of a kind lands beside the first
release_height_m = 0.10                        # tool height at release: the ore falls into the bay from here
release_wait_s = 1.0

# --- The ore package: from the eBot's deck into the crusher hopper ---
# Hopper rim (crusher collision mesh): x -1.251..-0.648, y -0.31..+0.30, z +0.207 in base_link.
# The release point sits inside the rim on the arm's side, where the arm can still reach;
# the package falls onto the funnel and slides in. y is just positive so the route from the
# deck sweeps across the left (rover) side, never over the conveyor.
hopper_drop_point = (-0.78, 0.02, 0.33)
package_travel_height_m = 0.40                 # the package hangs 0.076 m below the tool: clears the hopper rim and the rover
package_ready_point = (0.55, 0.05, 0.45)       # where the tool waits for the eBot: above the stop pose, clear of the deck
package_release_wait_s = 1.5

# --- Recovery from a protective stop ---
recovery_lift_speed_mps = 0.04
recovery_lift_s = 3.0                          # commanding time, inside the controller's 5 s recovery window
max_recoveries = 3                             # each protective stop costs marks: give up rather than repeat one

control_period_s = 0.05                        # 20 Hz: well inside the servo's 0.15 s dead-man

# --- Limits that turn a stalled step into a clear stop instead of a silent hang ---
ore_wait_s = 20.0                              # after this, start with the ores seen so far
ore_wait_limit_s = 90.0                        # with no ore at all after this, stop
sensor_wait_limit_s = 30.0                     # arm feedback missing for this long: stop
unfold_timeout_s = 40.0
motion_timeout_s = 90.0                        # one hover, lift or transit
max_target_shift_m = 0.03                      # an ore found this far from where it was planned: look again
release_retry_s = 3.0                          # extra wait for an ore that sticks to the tool


##################### HELPER FUNCTIONS #######################

def arc_points(start_xy, end_xy, radius, step_deg):
    '''
    Description:    Points on a circle round the base axis, between the azimuths of start_xy
                    and end_xy. The sweep is end minus start, so an ore on the right (negative
                    y) and a bay on the left (positive y) are joined across the front of the
                    base, azimuth 0, and never behind it, where the crusher stands.

    Returns:
        list of (x, y) strictly between the two azimuths, every one on the circle.
    '''

    start = math.atan2(start_xy[1], start_xy[0])
    sweep = math.atan2(end_xy[1], end_xy[0]) - start
    count = max(int(abs(sweep) / math.radians(step_deg)), 1)
    return [(radius * math.cos(start + sweep * i / count), radius * math.sin(start + sweep * i / count))
            for i in range(1, count)]


def clearance_height(x, y, low_height, ebot_near):
    '''
    Description:    The tool height to travel at over (x, y): low_height, raised to
                    front_clear_height_m in the sector in front of the base where the eBot
                    stops, while the eBot is near.
    '''

    in_front = x > 0.0 and abs(math.atan2(y, x)) < front_sector_rad
    return max(low_height, front_clear_height_m) if in_front and ebot_near else low_height


##################### CLASS DEFINITION #######################

class ArmFeedback:
    '''
    ___CLASS___

    Description:    The latest readings from the arm: where the tool is, the joint angles, the
                    safety state and the force at the tool.
    '''

    def __init__(self):
        self.tcp_position = None                  # ndarray (x, y, z) in base_link
        self.tcp_rotation = None                  # Rotation of the tool in base_link
        self.joint_angles = None                  # dict joint name -> radians
        self.status = None                        # arm_status code
        self.forces = deque(maxlen=force_window)  # force_magnitude samples, newest last

    def update_tcp(self, pose):
        p, q = pose.position, pose.orientation
        self.tcp_position = np.array([p.x, p.y, p.z])
        self.tcp_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])

    def update_joints(self, message):
        '''Joint angles by name: /joint_states is in alphabetical order, not chain order.'''
        self.joint_angles = dict(zip(message.name, message.position))

    @property
    def missing(self):
        '''Names of the readings that have not arrived yet.'''
        readings = {'tcp pose': self.tcp_position, 'joint states': self.joint_angles,
                    'arm status': self.status, 'force': self.forces or None}
        return [name for name, value in readings.items() if value is None]

    @property
    def ready(self):
        return not self.missing

    @property
    def faulted(self):
        return self.status >= status_protective_stop_min

    @property
    def force(self):
        '''Force at the tool, tool weight already removed by the arm: about 0 N in free air.'''
        return self.forces[-1]

    @property
    def contact_force(self):
        '''Force averaged over the last few samples, for contact decisions.'''
        return float(np.mean(list(self.forces)[-contact_window:]))

    @property
    def mean_force(self):
        return float(np.mean(self.forces))


@dataclass(frozen=True)
class Waypoint:
    '''
    ___CLASS___

    Description:    A point for the tool to reach, in base_link, and how to reach it.
    '''

    position: np.ndarray
    speed: float
    tolerance: float = travel_tolerance_m
    settle: bool = False                          # stand still for settle_s on arriving
    through: bool = False                         # pass it at full speed: a corner of a longer route


class CartesianServo:
    '''
    ___CLASS___

    Description:    Proportional control of the tool towards a waypoint: a linear velocity
                    along the line to it, and an angular velocity that holds the orientation.
                    Both are scaled as whole vectors, so the direction survives the speed caps.
    '''

    def command(self, feedback, waypoint, reference_rotation):
        '''
        Returns:
            (linear ndarray, angular ndarray, distance to the waypoint in metres)
        '''

        error = waypoint.position - feedback.tcp_position
        distance = float(np.linalg.norm(error))
        speed = min(waypoint.speed, cap_linear_mps)
        if waypoint.through and distance > 1.0e-6:
            linear = error * (speed / distance)   # no slowing down at a corner
        else:
            linear = self._limit(position_kp * error, speed)

        rotation_error = (reference_rotation * feedback.tcp_rotation.inv()).as_rotvec()
        angular = self._limit(orientation_kp * rotation_error, cap_angular_rps)
        return linear, angular, distance

    @staticmethod
    def _limit(vector, limit):
        magnitude = float(np.linalg.norm(vector))
        return vector * (limit / magnitude) if magnitude > limit else vector


class RouteFollower:
    '''
    ___CLASS___

    Description:    Drives the tool through a list of waypoints one after another. A waypoint
                    with settle=True is held for settle_s once reached; the others hand over to
                    the next one at once, so a route of many points is one smooth motion.
    '''

    def __init__(self, waypoints, servo):
        self.waypoints = list(waypoints)
        self.servo = servo
        self.index = 0
        self._settled_since = None

    @property
    def finished(self):
        return self.index >= len(self.waypoints)

    def step(self, feedback, reference_rotation, now_s):
        '''
        Returns:
            (linear, angular) to publish this cycle; zeros once the route is finished.
        '''

        zero = np.zeros(3)
        while not self.finished:
            waypoint = self.waypoints[self.index]
            linear, angular, distance = self.servo.command(feedback, waypoint, reference_rotation)
            if distance > waypoint.tolerance:
                self._settled_since = None
                return linear, angular
            if waypoint.settle:
                if self._settled_since is None:
                    self._settled_since = now_s
                if now_s - self._settled_since < settle_s:
                    return zero, angular
            self.index += 1
            self._settled_since = None
        return zero, zero


class JointUnfolder:
    '''
    ___CLASS___

    Description:    Moves the arm from its folded start pose to a pose with the elbow in the
                    middle of its range, in joint space, before any task-space motion.
    '''

    def velocities(self, joint_angles):
        '''
        Returns:
            dict joint name -> rad/s, or None once every joint is inside the tolerance.
        '''

        errors = {name: target - joint_angles[name] for name, target in unfold_target.items()}
        if max(abs(e) for e in errors.values()) <= unfold_tolerance_rad:
            return None
        velocities = {name: unfold_kp * e for name, e in errors.items()}
        scale = min(1.0, cap_joint_rps / max(abs(v) for v in velocities.values()))
        return {name: v * scale for name, v in velocities.items()}   # one scale for all: the joint path stays straight


class ContainerMap:
    '''
    ___CLASS___

    Description:    Where to release each kind of ore, in base_link. The bays are fixed in the
                    world, so their positions are converted once with the world-to-base transform.
    '''

    def __init__(self, tf_buffer):
        self.tf_buffer = tf_buffer

    def drop_point(self, kind, slot=0):
        '''
        Args:
            kind (str):     ore kind
            slot (int):     how many ores of this kind are already in the bay

        Returns:
            The release point (x, y, z) in base_link, or None until the world-to-base
            transform is available.
        '''

        try:
            stamped = self.tf_buffer.lookup_transform(base_frame, world_frame, rclpy.time.Time())
        except Exception:
            return None
        q, t = stamped.transform.rotation, stamped.transform.translation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        x, y = drop_points_world[kind]
        x += drop_slot_offsets_m[min(slot, len(drop_slot_offsets_m) - 1)]
        point = rotation.apply([x, y, 0.0]) + np.array([t.x, t.y, t.z])
        return np.array([point[0], point[1], release_height_m])


class ItemLocator:
    '''
    ___CLASS___

    Description:    Reads the item transforms the perception node broadcasts in base_link:
                    <kind>_ore_<n> for the ores and ore_package for the package on the eBot.
    '''

    def __init__(self, tf_buffer):
        self.tf_buffer = tf_buffer

    def locate(self, name):
        '''The middle (x, y, z) of one item, or None while its transform is unknown.'''
        try:
            t = self.tf_buffer.lookup_transform(base_frame, name, rclpy.time.Time()).transform.translation
        except Exception:
            return None
        return np.array([t.x, t.y, t.z])

    def scan(self):
        '''
        Returns:
            dict ore name -> ndarray (x, y, z) of its middle, for every ore currently known.
        '''

        found = {}
        for kind in ore_kinds:
            for number in range(1, ore_slots_per_kind + 1):
                name = f'{kind}_ore_{number}'
                position = self.locate(name)
                if position is not None:
                    found[name] = position
        return found

    @staticmethod
    def kind_of(name):
        return name.split('_ore_')[0]


class ServiceCaller:
    '''
    ___CLASS___

    Description:    One service client with at most one call in flight. The timer polls it, it
                    never blocks: spinning inside a callback of a node that is already
                    spinning hangs.
    '''

    def __init__(self, node, service_type, name):
        self.client = node.create_client(service_type, name)
        self._future = None

    @property
    def available(self):
        return self.client.service_is_ready()

    @property
    def pending(self):
        return self._future is not None and not self._future.done()

    def call(self, request):
        self._future = self.client.call_async(request)

    def take_result(self):
        '''The reply of the finished call, or None if the call is still pending or none was made.'''
        if self._future is None or not self._future.done():
            return None
        result, self._future = self._future.result(), None
        return result


class EbotState(Enum):
    '''What the planning node reports on /handover/ebot.'''

    EN_ROUTE = 'EN_ROUTE'
    AT_ARM = 'AT_ARM'                             # standing still beside the arm, package on the deck
    LEAVING = 'LEAVING'


class ArmState(Enum):
    '''What this node reports on /handover/arm.'''

    IDLE = 'IDLE'
    TAKING = 'TAKING'                             # the package job has started
    PACKAGE_CLEAR = 'PACKAGE_CLEAR'               # the package is held and lifted off the deck: the eBot may leave


class Phase(Enum):
    '''Where the arm is in the task.'''

    WAIT_FOR_ORES = 1
    SWITCH_TO_JOINT = 2
    UNFOLD = 3
    SWITCH_TO_TWIST = 4
    SELECT = 5
    HOVER = 6
    DESCEND = 7
    GRAB = 8
    LIFT = 9
    VERIFY = 10
    TRANSIT = 11
    RELEASE = 12
    WAIT_FOR_PACKAGE = 13
    RECOVER_UNLOCK = 14
    RECOVER_LIFT = 15
    FINISHED = 16
    FAILED = 17


class ore_manipulation(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to pick each ore off the conveyor with the
                    magnet and drop it in the container for its kind, and to take the ore
                    package off the eBot's deck and drop it in the crusher hopper, without a
                    protective stop. The package is taken at the first job boundary after the
                    eBot reports that it stands beside the arm.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_manipulation
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ore_manipulation_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)   # tool velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)       # per-joint velocity
        self.arm_state_pub = self.create_publisher(String, arm_state_topic, latched)    # handover state for the eBot

        ############ Topic SUBSCRIPTIONS ############

        self.ebot_state_sub = self.create_subscription(String, ebot_state_topic, self.ebotstatecb, latched)
        self.ebot_odom_sub = self.create_subscription(Odometry, '/odom', self.ebotodomcb, 10)
        self.feedback = ArmFeedback()
        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(
            Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 50)

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.items = ItemLocator(self.tf_buffer)
        self.containers = ContainerMap(self.tf_buffer)
        self.servo = CartesianServo()
        self.unfolder = JointUnfolder()
        self.switcher = ServiceCaller(self, SwitchController, f'{servo_ns}/switch_controller')
        self.unlocker = ServiceCaller(self, Trigger, f'{servo_ns}/unlock_protective_stop')
        self.magnet = ServiceCaller(self, SetBool, '/magnet')

        self.phase = Phase.WAIT_FOR_ORES
        self.phase_since_s = None                 # set on the first control cycle, when the sim clock is running
        self.route = None                         # RouteFollower of the current motion
        self.reference_rotation = None            # tool-down orientation held for the whole run
        self.target_name = None                   # item being handled: an ore name or package_frame
        self.target_position = None               # its middle, in base_link
        self.attempts = {}                        # item name -> picks tried
        self.delivered = set()                    # ores in their containers
        self.package_done = False                 # the package is in the hopper, or every try at it failed
        self.ebot_state = None                    # last EbotState from the planning node
        self.ebot_xy = None                       # where the eBot is, map frame
        self.arm_state = None
        self.recoveries = 0
        self.waiting_since_s = None               # when the arm feedback was first found incomplete
        self.grab_started_s = None
        self.last_status = None
        self.handlers = {
            Phase.WAIT_FOR_ORES: self.wait_for_ores,
            Phase.SWITCH_TO_JOINT: self.switch_to_joint,
            Phase.UNFOLD: self.unfold,
            Phase.SWITCH_TO_TWIST: self.switch_to_twist,
            Phase.SELECT: self.select_ore,
            Phase.HOVER: self.hover,
            Phase.DESCEND: self.descend,
            Phase.GRAB: self.grab,
            Phase.LIFT: self.lift,
            Phase.VERIFY: self.verify_hold,
            Phase.TRANSIT: self.transit,
            Phase.RELEASE: self.release,
            Phase.WAIT_FOR_PACKAGE: self.wait_for_package,
            Phase.RECOVER_UNLOCK: self.recover_unlock,
            Phase.RECOVER_LIFT: self.recover_lift,
            Phase.FINISHED: self.idle,
            Phase.FAILED: self.idle,
        }
        self.timer = self.create_timer(control_period_s, self.control_cycle)
        self.report_arm_state(ArmState.IDLE)

    ############ Callbacks ############

    def ebotstatecb(self, data):
        '''
        Description:    Callback function for the eBot's handover state.

        Args:
            data (String):    One EbotState token, published by the planning node
        '''

        try:
            state = EbotState(data.data)
        except ValueError:
            self.get_logger().warn(f'Ignoring unknown eBot state "{data.data}"')
            return
        if state is not self.ebot_state:
            self.ebot_state = state
            self.get_logger().info(f'eBot: {state.value}')

    def ebotodomcb(self, data):
        '''
        Description:    Callback function for the eBot's odometry: only its place matters here.

        Args:
            data (Odometry):    Pose and velocity of the eBot
        '''

        self.ebot_xy = (data.pose.pose.position.x, data.pose.pose.position.y)

    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link
        '''

        self.feedback.update_tcp(data.pose)

    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.

        Args:
            data (JointState):    Joint feedback published by the arm
        '''

        self.feedback.update_joints(data)

    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.

        Args:
            data (Int32):    One state code describing what the arm is doing
        '''

        self.feedback.status = data.data
        if data.data != self.last_status:
            self.last_status = data.data
            self.get_logger().info(f'arm_status {data.data}')

    def forcecb(self, data):
        '''
        Description:    Callback function for the force at the tool.

        Args:
            data (Float32):    External force at the tool in newtons, tool weight removed
        '''

        self.feedback.forces.append(float(data.data))

    ############ Helpers ############

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def enter(self, phase):
        '''Change phase and restart its timer.'''
        self.get_logger().info(f'{self.phase.name} -> {phase.name}')
        self.phase = phase
        self.phase_since_s = self.now_s()

    def in_phase_for(self):
        if self.phase_since_s is None:            # the world may have run for minutes before this node started
            self.phase_since_s = self.now_s()
        return self.now_s() - self.phase_since_s

    def publish_twist(self, linear, angular):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = base_frame
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = (float(v) for v in linear)
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = (float(v) for v in angular)
        self.twist_pub.publish(message)

    def stop(self):
        self.publish_twist(np.zeros(3), np.zeros(3))

    def publish_joints(self, velocities):
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(velocities)
        message.velocities = [float(v) for v in velocities.values()]
        self.joint_pub.publish(message)

    def request_magnet(self, energise):
        request = SetBool.Request()
        request.data = energise
        self.magnet.call(request)

    def request_controller(self, activate, deactivate):
        request = SwitchController.Request()
        request.activate_controllers = [activate]
        request.deactivate_controllers = [deactivate]
        request.strictness = SwitchController.Request.STRICT
        self.switcher.call(request)

    def drive(self, route):
        '''Publish one cycle of a route. Returns True once the route has finished.'''
        linear, angular = route.step(self.feedback, self.reference_rotation, self.now_s())
        self.publish_twist(linear, angular)
        return route.finished

    def report_arm_state(self, state):
        '''Publish the handover state once per change; the topic is latched.'''
        if state is self.arm_state:
            return
        self.arm_state = state
        self.arm_state_pub.publish(String(data=state.value))
        self.get_logger().info(f'Handover: arm {state.value}')

    def ebot_near_arm(self):
        '''True while the eBot is in, or on its way into or out of, the stop sector in front of the arm.'''
        if self.ebot_state is EbotState.AT_ARM or self.ebot_xy is None:
            return True
        return self.ebot_xy[0] < ebot_near_max_x_m and self.ebot_xy[1] > ebot_near_min_y_m

    @property
    def handling_package(self):
        return self.target_name == package_frame

    def target_top(self):
        '''Middle of the face the magnet lands on: the top of the target item.'''
        return self.target_position + np.array([0.0, 0.0, item_height_m / 2.0])

    def transit_route(self, destination, destination_speed, low_height=travel_height_m):
        '''
        Route from the tool's present place to destination (x, y, z) along an arc round the
        base at low_height, raised over the eBot's stop sector in front of the base. Items and
        their drop points lie on different sides of the base, and a straight line between them
        would pass through the arm's singular region at the base axis.
        '''

        start = self.feedback.tcp_position
        near = self.ebot_near_arm()
        waypoints = []
        if math.hypot(start[0], start[1]) > 0.3:
            lift = np.array([start[0], start[1], max(start[2], clearance_height(*start[:2], low_height, near))])
            waypoints.append(Waypoint(lift, lift_speed_mps * 2.0, pass_tolerance_m, through=True))
            for x, y in arc_points(start[:2], destination[:2], swing_radius_m, swing_step_deg):
                waypoints.append(Waypoint(np.array([x, y, clearance_height(x, y, low_height, near)]),
                                          transit_speed_mps, pass_tolerance_m, through=True))
        above = clearance_height(destination[0], destination[1], low_height, near)
        waypoints.append(Waypoint(np.array([destination[0], destination[1], max(above, destination[2])]),
                                  transit_speed_mps))
        waypoints.append(Waypoint(np.asarray(destination, dtype=float), destination_speed,
                                  precise_tolerance_m, settle=True))
        return RouteFollower(waypoints, self.servo)

    ############ State machine ############

    def control_cycle(self):
        '''
        Description:    Timer function: one step of the state machine. A protective stop
                        interrupts whatever the arm is doing.
        '''

        if self.now_s() <= 0.0:                   # no /clock received yet: every timer would start at zero
            return
        self.check_magnet_reply()
        if not self.feedback.ready:
            self.wait_for_feedback()
            return
        self.waiting_since_s = None
        recovering = self.phase in (Phase.RECOVER_UNLOCK, Phase.RECOVER_LIFT,
                                    Phase.FINISHED, Phase.FAILED)
        motion_started = self.reference_rotation is not None
        if self.feedback.faulted and not recovering and motion_started:
            self.begin_recovery()
            return
        if self.timed_out():
            return
        self.handlers[self.phase]()

    def wait_for_feedback(self):
        '''The arm has not reported everything yet: give it a time, then stop with the reason.'''
        if self.waiting_since_s is None:
            self.waiting_since_s = self.now_s()
        if self.phase is not Phase.FAILED and self.now_s() - self.waiting_since_s > sensor_wait_limit_s:
            self.fail(f'no {", ".join(self.feedback.missing)} from the arm after {sensor_wait_limit_s:.0f} s')

    def check_magnet_reply(self):
        reply = self.magnet.take_result()
        if reply is not None and not reply.success:
            self.get_logger().warn(f'magnet service refused the request: {reply.message}')

    def timed_out(self):
        '''Stop a step that has taken far longer than any normal one. Returns True if it acted.'''
        taken = self.in_phase_for()
        if self.phase is Phase.UNFOLD and taken > unfold_timeout_s:
            self.fail('the arm did not unfold')
        elif self.phase is Phase.HOVER and taken > motion_timeout_s:
            self.get_logger().warn(f'{self.target_name} could not be reached in {motion_timeout_s:.0f} s: skipping this try')
            self.begin_abort_pick()
        elif self.phase in (Phase.LIFT, Phase.TRANSIT) and taken > motion_timeout_s:
            self.fail(f'{self.phase.name} did not finish in {motion_timeout_s:.0f} s')
        else:
            return False
        return True

    def idle(self):
        self.stop()

    def wait_for_ores(self):
        '''Wait until the perception node has published every ore, or give it a fixed time.'''
        known = self.items.scan()
        waited = self.in_phase_for()
        if len(known) < expected_ore_count and not (known and waited > ore_wait_s):
            if waited > ore_wait_limit_s:
                self.fail('no ore transform from the perception node')
            return
        self.get_logger().info(f'Ores known: {sorted(known)}')
        self.request_controller(joint_controller, twist_controller)
        self.enter(Phase.SWITCH_TO_JOINT)

    def switch_to_joint(self):
        reply = self.switcher.take_result()
        if reply is None:
            return
        if reply.ok:
            self.enter(Phase.UNFOLD)
        else:
            self.fail('could not switch to the joint controller')

    def unfold(self):
        velocities = self.unfolder.velocities(self.feedback.joint_angles)
        if velocities is not None:
            self.publish_joints(velocities)
            return
        self.publish_joints({name: 0.0 for name in unfold_target})
        self.reference_rotation = self.vertical_tool_rotation(self.feedback.tcp_rotation)
        self.request_controller(twist_controller, joint_controller)
        self.enter(Phase.SWITCH_TO_TWIST)

    @staticmethod
    def vertical_tool_rotation(rotation):
        '''The given orientation, turned by the smallest angle that points the tool axis straight down.'''
        axis = rotation.apply([0.0, 0.0, 1.0])
        down = np.array([0.0, 0.0, -1.0])
        turn_axis = np.cross(axis, down)
        sine = np.linalg.norm(turn_axis)
        if sine < 1e-6:
            return rotation if axis @ down > 0.0 else Rotation.from_rotvec([np.pi, 0.0, 0.0]) * rotation
        angle = np.arctan2(sine, axis @ down)
        return Rotation.from_rotvec(turn_axis / sine * angle) * rotation

    def switch_to_twist(self):
        reply = self.switcher.take_result()
        if reply is None:
            return
        if reply.ok:
            self.enter(Phase.SELECT)
        else:
            self.fail('could not switch to the twist controller')

    def select_ore(self):
        '''
        Choose the next job. The package comes first once the eBot stands beside the arm and
        the package has been seen, so the eBot can leave as early as possible; otherwise the
        next ore still on the conveyor, the one furthest forward.
        '''
        self.stop()
        if self.package_waiting():
            self.begin_pick(package_frame, self.items.locate(package_frame))
            return
        positions = self.items.scan()
        remaining = {name: p for name, p in positions.items()
                     if name not in self.delivered and self.attempts.get(name, 0) < max_pick_attempts}
        if remaining:
            # Ores not yet tried come first, so one that keeps failing cannot hold up the rest.
            name = min(remaining, key=lambda n: (self.attempts.get(n, 0), -remaining[n][0]))
            self.begin_pick(name, remaining[name])
            return
        if not self.package_done:
            self.route = self.transit_route(np.array(package_ready_point), approach_speed_mps)
            self.enter(Phase.WAIT_FOR_PACKAGE)
            return
        self.get_logger().info(f'Done: {len(self.delivered)} ore(s) delivered: {sorted(self.delivered)}, '
                               f'package in the hopper')
        self.enter(Phase.FINISHED)

    def package_waiting(self):
        '''True when the package is on the eBot beside the arm, seen, and still to be taken.'''
        if self.package_done or self.ebot_state is not EbotState.AT_ARM:
            return False
        if self.attempts.get(package_frame, 0) >= max_pick_attempts:
            self.get_logger().error('The package could not be picked: leaving it on the eBot')
            self.package_done = True
            return False
        return self.items.locate(package_frame) is not None

    def begin_pick(self, name, position):
        '''Start the move to hover above an item.'''
        self.target_name = name
        self.target_position = position
        self.attempts[name] = self.attempts.get(name, 0) + 1
        if self.handling_package:
            self.report_arm_state(ArmState.TAKING)
        hover = self.target_top() + np.array([0.0, 0.0, hover_clearance_m])
        self.route = self.transit_route(hover, approach_speed_mps)
        self.get_logger().info(
            f'Picking {name} at {np.round(position, 3)} (attempt {self.attempts[name]})')
        self.enter(Phase.HOVER)

    def wait_for_package(self):
        '''Every ore is done: wait above the eBot's stop pose for the package to arrive.'''
        self.drive(self.route)
        if self.package_waiting():
            self.enter(Phase.SELECT)

    def hover(self):
        if not self.drive(self.route):
            return
        fresh = self.items.locate(self.target_name)                       # last look before the descent
        if fresh is not None:
            if np.linalg.norm(fresh[:2] - self.target_position[:2]) > max_target_shift_m:
                self.get_logger().warn(f'{self.target_name} is not where it was planned: looking again')
                self.begin_abort_pick()
                return
            self.target_position = fresh
        self.request_magnet(True)
        column = self.target_position[:2]
        top_z = self.target_top()[2]
        start = np.array([*column, top_z + contact_start_m])
        goal = np.array([*column, top_z - search_depth_m])
        self.route = RouteFollower([Waypoint(start, descent_speed_mps, precise_tolerance_m),
                                    Waypoint(goal, contact_speed_mps, contact_tolerance_m)], self.servo)
        self.enter(Phase.DESCEND)

    def descend(self):
        if self.feedback.contact_force > overpress_force_n:
            self.get_logger().warn(f'Pressing too hard ({self.feedback.contact_force:.2f} N): backing off')
            self.begin_abort_pick()
            return
        above_top_m = self.feedback.tcp_position[2] - self.target_top()[2]
        touched = self.feedback.contact_force > touch_force_n and above_top_m < touch_zone_m
        arrived = self.drive(self.route)
        if touched or arrived or self.in_phase_for() > descent_timeout_s:
            reason = 'touch' if touched else 'no contact' if arrived else 'timeout'
            self.get_logger().info(
                f'Descent ended ({reason}): tool {above_top_m * 1000:.1f} mm above the top face, '
                f'force {self.feedback.force:.2f} N, after {self.in_phase_for():.1f} s')
            self.stop()
            self.grab_started_s = self.now_s()
            self.enter(Phase.GRAB)

    def grab(self):
        self.stop()
        if self.now_s() - self.grab_started_s < grab_wait_s:
            return
        hover = self.target_top() + np.array([0.0, 0.0, hover_clearance_m])
        hover[:2] = self.feedback.tcp_position[:2]
        self.route = RouteFollower([Waypoint(hover, lift_speed_mps, precise_tolerance_m, settle=True)], self.servo)
        self.enter(Phase.LIFT)

    def lift(self):
        if self.drive(self.route):
            self.enter(Phase.VERIFY)

    def verify_hold(self):
        '''The item on the tool adds its weight to the force reading; free air reads about 0 N.'''
        self.stop()
        if self.in_phase_for() < 0.4:
            return
        low, high = package_holding_force_n if self.handling_package else holding_force_n
        if not low <= self.feedback.mean_force <= high:
            self.get_logger().warn(f'{self.target_name} not held (force {self.feedback.mean_force:.2f} N)')
            self.begin_abort_pick()
            return
        self.get_logger().info(f'{self.target_name} held ({self.feedback.mean_force:.2f} N)')
        if self.handling_package:
            self.report_arm_state(ArmState.PACKAGE_CLEAR)                # lifted off the deck: the eBot may go
            self.route = self.transit_route(np.array(hopper_drop_point), approach_speed_mps,
                                            package_travel_height_m)
        else:
            kind = self.items.kind_of(self.target_name)
            slot = sum(1 for name in self.delivered if self.items.kind_of(name) == kind)
            drop = self.containers.drop_point(kind, slot)
            if drop is None:
                return
            self.route = self.transit_route(drop, approach_speed_mps)
        self.enter(Phase.TRANSIT)

    def begin_abort_pick(self):
        '''Let the item go and go round again: the next pick re-reads where it lies.'''
        self.request_magnet(False)
        self.enter(Phase.SELECT)

    def transit(self):
        if self.drive(self.route):
            self.request_magnet(False)
            self.enter(Phase.RELEASE)

    def release(self):
        self.stop()
        waited = self.in_phase_for()
        wait_s = package_release_wait_s if self.handling_package else release_wait_s
        if waited < wait_s:
            return
        sticking = self.feedback.mean_force > holding_force_n[0]
        if sticking and waited < wait_s + release_retry_s:
            return
        if sticking:
            self.get_logger().warn(f'{self.target_name} still reads on the tool after release')
        if self.handling_package:
            self.package_done = True
            self.get_logger().info('Ore package dropped in the crusher hopper')
        else:
            self.delivered.add(self.target_name)
            self.get_logger().info(f'{self.target_name} delivered')
        self.enter(Phase.SELECT)

    ############ Recovery ############

    def begin_recovery(self):
        '''A protective stop latched the arm: release the ore, unlock, then lift away.'''
        self.get_logger().warn(f'Protective stop (arm_status {self.feedback.status}) while in {self.phase.name}')
        self.stop()
        self.recoveries += 1
        if self.recoveries > max_recoveries:
            self.fail('too many protective stops')
            return
        self.request_magnet(False)
        self.unlocker.call(Trigger.Request())
        self.enter(Phase.RECOVER_UNLOCK)

    def recover_unlock(self):
        self.stop()
        reply = self.unlocker.take_result()
        if reply is None:
            return
        self.get_logger().info(f'unlock: {reply.message}')
        self.route = RouteFollower(
            [Waypoint(self.feedback.tcp_position + np.array([0.0, 0.0, 0.5]), recovery_lift_speed_mps,
                      precise_tolerance_m)], self.servo)
        self.enter(Phase.RECOVER_LIFT)

    def recover_lift(self):
        '''Only motion that reduces the violation is accepted, and only for 5 s of commanding.'''
        self.drive(self.route)
        if self.in_phase_for() < recovery_lift_s:
            return
        self.stop()
        if self.feedback.faulted:
            self.fail(f'arm still latched (arm_status {self.feedback.status}) after recovery')
            return
        if self.handling_package and self.arm_state is ArmState.PACKAGE_CLEAR:
            # The magnet let go off the eBot's deck: the package is no longer anywhere it can be found.
            self.get_logger().error('The package was released during recovery: it cannot be picked again')
            self.package_done = True
        self.enter(Phase.SELECT)

    def fail(self, reason):
        self.get_logger().error(f'Stopping: {reason}')
        self.stop()
        self.request_magnet(False)
        self.enter(Phase.FAILED)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ore_manipulation class to perform its task
    '''

    rclpy.init(args=sys.argv)                                                           # initialisation

    ore_manipulation_class = ore_manipulation()                                         # creating a new object for class 'ore_manipulation'

    try:
        rclpy.spin(ore_manipulation_class)                                              # spinning on the object to make it alive in ROS 2 DDS
    except KeyboardInterrupt:
        ore_manipulation_class.stop()                                                   # leave the arm standing still
    finally:
        ore_manipulation_class.destroy_node()                                           # destroy node after spin ends
        if rclpy.ok():
            rclpy.shutdown()                                                            # shutdown process


if __name__ == '__main__':

    main()
