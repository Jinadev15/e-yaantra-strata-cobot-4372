#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 2A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task2A_manipulation.py
# Functions:
#			        arc_points, main
# Classes:          ArmFeedback, Waypoint, CartesianServo, RouteFollower, JointUnfolder,
#                   ContainerMap, OreLocator, ServiceCaller, Phase, ore_manipulation
# Nodes:		    ore_manipulation_node
#                   Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /arm_status,
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
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
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

# What the servo accepts: a command above one of these is dropped whole, not clamped.
cap_linear_mps = 0.12
cap_angular_rps = 0.30
cap_joint_rps = 0.30

# Arm status codes (The UR7e Arm): 0 is healthy, 10-14 warn, 20-23 are latched protective stops.
status_protective_stop_min = 20

ore_kinds = ('azurite', 'malachite', 'vanadinite')
ore_slots_per_kind = 4                         # an ore of one kind is numbered 1 up to this
ore_height_m = 0.0762                          # ore block height; the transform names its middle
expected_ore_count = 4

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
transit_speed_mps = 0.11
approach_speed_mps = 0.08
descent_speed_mps = 0.025                      # the arm coasts about 5 mm at 0.02 m/s and 50 mm at 0.10 m/s
lift_speed_mps = 0.05
travel_tolerance_m = 0.02
precise_tolerance_m = 0.008
settle_s = 0.4                                 # stand still this long on arriving at a precise waypoint

# --- Heights, in base_link ---
hover_clearance_m = 0.12                       # tool face above an ore's top face before the descent
travel_height_m = 0.14                         # tool height while carrying: clears the bay rim by 0.1 m with the ore hanging below
swing_radius_m = 0.60                          # radius of the arc round the front of the base
swing_step_deg = 30.0

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
force_window = 10                              # samples averaged to judge the hold
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
        linear = self._limit(position_kp * error, min(waypoint.speed, cap_linear_mps))

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


class OreLocator:
    '''
    ___CLASS___

    Description:    Reads the ore transforms the perception node broadcasts
                    (<kind>_ore_<n> in base_link) from the tf buffer.
    '''

    def __init__(self, tf_buffer):
        self.tf_buffer = tf_buffer

    def scan(self):
        '''
        Returns:
            dict ore name -> ndarray (x, y, z) of its middle, for every ore currently known.
        '''

        found = {}
        for kind in ore_kinds:
            for number in range(1, ore_slots_per_kind + 1):
                name = f'{kind}_ore_{number}'
                try:
                    t = self.tf_buffer.lookup_transform(base_frame, name, rclpy.time.Time()).transform.translation
                except Exception:
                    continue
                found[name] = np.array([t.x, t.y, t.z])
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
    RECOVER_UNLOCK = 13
    RECOVER_LIFT = 14
    FINISHED = 15
    FAILED = 16


class ore_manipulation(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to pick each ore off the conveyor with the
                    magnet and drop it in the container for its kind, without a protective stop.
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

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)   # tool velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)       # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.feedback = ArmFeedback()
        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(
            Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 50)

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ores = OreLocator(self.tf_buffer)
        self.containers = ContainerMap(self.tf_buffer)
        self.servo = CartesianServo()
        self.unfolder = JointUnfolder()
        self.switcher = ServiceCaller(self, SwitchController, f'{servo_ns}/switch_controller')
        self.unlocker = ServiceCaller(self, Trigger, f'{servo_ns}/unlock_protective_stop')
        self.magnet = ServiceCaller(self, SetBool, '/magnet')

        self.phase = Phase.WAIT_FOR_ORES
        self.phase_since_s = 0.0
        self.route = None                         # RouteFollower of the current motion
        self.reference_rotation = None            # tool-down orientation held for the whole run
        self.target_name = None                   # ore being handled
        self.target_position = None               # its middle, in base_link
        self.attempts = {}                        # ore name -> picks tried
        self.delivered = set()
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
            Phase.RECOVER_UNLOCK: self.recover_unlock,
            Phase.RECOVER_LIFT: self.recover_lift,
            Phase.FINISHED: self.idle,
            Phase.FAILED: self.idle,
        }
        self.timer = self.create_timer(control_period_s, self.control_cycle)

    ############ Callbacks ############

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

    def ore_top(self):
        '''Middle of the face the magnet lands on: the top of the target ore.'''
        return self.target_position + np.array([0.0, 0.0, ore_height_m / 2.0])

    def transit_route(self, destination, destination_speed):
        '''
        Route from the tool's present place to destination (x, y, z), at travel height across
        the front of the base. An ore and a bay lie on opposite sides of the base, and a straight
        line between them would pass through the arm's singular region at the base axis.
        '''

        start = self.feedback.tcp_position
        waypoints = []
        if math.hypot(start[0], start[1]) > 0.3:
            lift = np.array([start[0], start[1], max(start[2], travel_height_m)])
            waypoints.append(Waypoint(lift, lift_speed_mps * 2.0))
            for x, y in arc_points(start[:2], destination[:2], swing_radius_m, swing_step_deg):
                waypoints.append(Waypoint(np.array([x, y, travel_height_m]), transit_speed_mps))
        waypoints.append(Waypoint(np.array([destination[0], destination[1], travel_height_m]),
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
        known = self.ores.scan()
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
        '''Choose the next ore still on the conveyor: the one furthest forward, nearest the front of the base.'''
        self.stop()
        positions = self.ores.scan()
        remaining = {name: p for name, p in positions.items()
                     if name not in self.delivered and self.attempts.get(name, 0) < max_pick_attempts}
        if not remaining:
            self.get_logger().info(f'Done: {len(self.delivered)} ore(s) delivered: {sorted(self.delivered)}')
            self.enter(Phase.FINISHED)
            return
        # Ores not yet tried come first, so one that keeps failing cannot hold up the rest.
        self.target_name = min(remaining, key=lambda name: (self.attempts.get(name, 0), -remaining[name][0]))
        self.target_position = remaining[self.target_name]
        self.attempts[self.target_name] = self.attempts.get(self.target_name, 0) + 1
        hover = self.ore_top() + np.array([0.0, 0.0, hover_clearance_m])
        self.route = self.transit_route(hover, approach_speed_mps)
        self.get_logger().info(
            f'Picking {self.target_name} at {np.round(self.target_position, 3)} '
            f'(attempt {self.attempts[self.target_name]})')
        self.enter(Phase.HOVER)

    def hover(self):
        if not self.drive(self.route):
            return
        fresh = self.ores.scan().get(self.target_name)                   # last look before the descent
        if fresh is not None:
            if np.linalg.norm(fresh[:2] - self.target_position[:2]) > max_target_shift_m:
                self.get_logger().warn(f'{self.target_name} is not where it was planned: looking again')
                self.begin_abort_pick()
                return
            self.target_position = fresh
        self.request_magnet(True)
        column = self.target_position[:2]
        top_z = self.ore_top()[2]
        start = np.array([*column, top_z + contact_start_m])
        goal = np.array([*column, top_z - search_depth_m])
        self.route = RouteFollower([Waypoint(start, descent_speed_mps, precise_tolerance_m),
                                    Waypoint(goal, contact_speed_mps, contact_tolerance_m)], self.servo)
        self.enter(Phase.DESCEND)

    def descend(self):
        if self.feedback.force > overpress_force_n:
            self.get_logger().warn(f'Pressing too hard ({self.feedback.force:.2f} N): backing off')
            self.begin_abort_pick()
            return
        above_top_m = self.feedback.tcp_position[2] - self.ore_top()[2]
        touched = self.feedback.force > touch_force_n and above_top_m < touch_zone_m
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
        hover = self.ore_top() + np.array([0.0, 0.0, hover_clearance_m])
        hover[:2] = self.feedback.tcp_position[:2]
        self.route = RouteFollower([Waypoint(hover, lift_speed_mps, precise_tolerance_m, settle=True)], self.servo)
        self.enter(Phase.LIFT)

    def lift(self):
        if self.drive(self.route):
            self.enter(Phase.VERIFY)

    def verify_hold(self):
        '''An ore on the tool adds its weight to the force reading; free air reads about 0 N.'''
        self.stop()
        if self.in_phase_for() < 0.4:
            return
        low, high = holding_force_n
        if low <= self.feedback.mean_force <= high:
            self.get_logger().info(f'{self.target_name} held ({self.feedback.mean_force:.2f} N)')
            kind = self.ores.kind_of(self.target_name)
            slot = sum(1 for name in self.delivered if self.ores.kind_of(name) == kind)
            drop = self.containers.drop_point(kind, slot)
            if drop is None:
                return
            self.route = self.transit_route(drop, approach_speed_mps)
            self.enter(Phase.TRANSIT)
        else:
            self.get_logger().warn(f'{self.target_name} not held (force {self.feedback.mean_force:.2f} N)')
            self.begin_abort_pick()

    def begin_abort_pick(self):
        '''Let the ore go and go round again: the next pick re-reads where it lies.'''
        self.request_magnet(False)
        self.enter(Phase.SELECT)

    def transit(self):
        if self.drive(self.route):
            self.request_magnet(False)
            self.enter(Phase.RELEASE)

    def release(self):
        self.stop()
        waited = self.in_phase_for()
        if waited < release_wait_s:
            return
        sticking = self.feedback.mean_force > holding_force_n[0]
        if sticking and waited < release_wait_s + release_retry_s:
            return
        if sticking:
            self.get_logger().warn(f'{self.target_name} still reads on the tool after release')
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
        else:
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
