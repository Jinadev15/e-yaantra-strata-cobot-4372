#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
import time
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from scipy.spatial.transform import Rotation

##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.12     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')
        self.timer = self.create_timer(control_rate, self.process_waypoints)            # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your motion needs to keep between cycles.
        #       ->  HINT: Which waypoint you are on, and what the arm is doing about it.

        ############################################
        self.current_waypoint = 0
        self.hold_start_time = None
        self.target_orientation = None
        self.position_kp = 1.5
        self.orientation_kp = 1.0
        self.position_tolerance = 0.015
        self.orientation_tolerance = 0.05

        self.startup_phase = 'ESCAPE'
        # Mid-range pose from the UR7e manual (elbow away from its -160 deg limit,
        # wrist away from the start-pose singularity). Reached in joint space.
        self.unfold_target = {
            'shoulder_lift_joint': -1.20,
            'elbow_joint': -1.60,
            'wrist_1_joint': -1.90,
        }
        self.unfold_kp = 0.8                 # 1/s, joint error -> velocity
        self.unfold_max_speed = 0.30         # rad/s, below the 0.35 rad/s input cap
        self.unfold_tolerance = 0.02         # rad
        # Tilt the tool this many degrees (negative = tip pointing outward, away from
        # the base). Keeps the wrist closer to the base so the arm never stretches
        # straight at the far waypoints, which is a singularity.
        self.tool_tilt_deg = -25.0
        self.last_logged_status = None
        self.switch_future = None
        self.controller_ready = True
        self.switch_controller("joint_controller")
    def tcpposecb(self, data):
        '''
       Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.tcp_pose = data.pose
        # INSTRUCTIONS & HELP :

        #	->  Store the tool's position and orientation.
        #       ->  HINT: data.pose.position, data.pose.orientation

        ############################################


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        try:
            self.joint_angles = [
                data.position[data.name.index(joint)]
                for joint in joint_names
            ]
        except ValueError:
            self.joint_angles = None
        # INSTRUCTIONS & HELP :

        #	->  Store the joint angles, matched BY NAME - the order is not promised.
        #       ->  HINT: angles = [data.position[data.name.index(j)] for j in joint_names]
        #       ->  NOTE: A joint assumed to be at zero when it is not reads as a large
        #                 error, and a controller acting on it drives hard towards it.

        ############################################


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.arm_status = data.data
        # INSTRUCTIONS & HELP :

        #	->  Store the code, and log it while you are developing. Zero is healthy; anything
        #       else is the arm telling you about the last command or its own state.
        #       ->  HINT: /arm_status_detail says the same thing in words.
        #       ->  NOTE: A protective stop LATCHES and cannot be cleared - the run is over.

        ############################################
    def switch_controller(self, controller):
        '''
        Switch between the joint and Cartesian controllers.
        Retries the request up to 5 times if the switch is rejected.
        '''

        if controller == 'twist_controller':
            activate = twist_controller
            deactivate = joint_controller

        elif controller == 'joint_controller':
            activate = joint_controller
            deactivate = twist_controller

        else:
            self.get_logger().error(
                f'Invalid controller name: {controller}'
            )
            return False

        if not self.switch_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                'Switch controller service not available.'
            )
            return False

        for attempt in range(1, 6):

            req = SwitchController.Request()
            req.activate_controllers = [activate]
            req.deactivate_controllers = [deactivate]
            req.strictness = SwitchController.Request.STRICT

            self.get_logger().info(
                f'Switching controller {deactivate} -> {activate} '
                f'(attempt {attempt}/5)'
            )

            future = self.switch_cli.call_async(req)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=10.0
            )

            result = future.result()

            if result is not None and result.ok:
                self.get_logger().info(
                    f'Controller activated: {activate}'
                )
                return True

            self.get_logger().warn(
                f'Controller switch attempt {attempt} failed.'
            )

            if attempt < 5:
                time.sleep(1.0)

        self.get_logger().error(
            f'Failed to activate controller after 5 attempts: {activate}'
        )
        return False

        # INSTRUCTIONS & HELP :

        #	->  Call the switch_controller service on self.switch_cli-
        #           req = SwitchController.Request()
        #           req.activate_controllers = [ the one you want ]
        #           req.deactivate_controllers = [ the other one ]
        #           req.strictness = SwitchController.Request.STRICT

        #   ->  Wait for it first, and expect the first attempt to fail-
        #           self.switch_cli.wait_for_service(timeout_sec=20.0)
        #           future = self.switch_cli.call_async(req)
        #           rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        #       ->  NOTE: That last call HANGS if made from a callback of a node that is
        #                 already spinning. __init__ is safe; from the timer, poll
        #                 'future.done()' instead.

        #   ->  Do not switch more often than you need to.

        ############################################
    def start_twist_controller_switch(self):
        """Request the switch without blocking the timer callback."""

        req = SwitchController.Request()
        req.activate_controllers = [twist_controller]
        req.deactivate_controllers = [joint_controller]
        req.strictness = SwitchController.Request.STRICT

        self.get_logger().info(
            'Requesting switch: delta_joint_controller -> delta_twist_controller'
        )

        self.switch_future = self.switch_cli.call_async(req)
    def tool_target_rotation(self, target):
        """
        Orientation the tool should have at the given target: the pose held after the
        unfold (tool pointing down), tilted outward about the horizontal axis that is
        perpendicular to the direction from the base to the target.
        """
        base = Rotation.from_quat([
            self.target_orientation.x,
            self.target_orientation.y,
            self.target_orientation.z,
            self.target_orientation.w
        ])

        radial = math.hypot(target[0], target[1])
        if radial < 1e-6:
            return base

        tilt = math.radians(self.tool_tilt_deg)
        axis_x = -target[1] / radial
        axis_y = target[0] / radial
        return Rotation.from_rotvec([axis_x * tilt, axis_y * tilt, 0.0]) * base

    def process_waypoints(self):
        '''
        Description: Drive the TCP through the required waypoints using
        feedback-based Cartesian velocity control.
        '''

        # ---------------------------------------------------------
        # STARTUP SINGULARITY ESCAPE
        # ---------------------------------------------------------
        if self.startup_phase == 'ESCAPE':

            if self.joint_angles is None or self.arm_status is None:
                return

            # Never start moving, or enter Cartesian control, during a fault.
            if self.arm_status >= 20:
                return

            names = list(self.unfold_target)
            errors = [
                self.unfold_target[n]
                - self.joint_angles[joint_names.index(n)]
                for n in names
            ]

            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = names

            if max(abs(e) for e in errors) > self.unfold_tolerance:
                velocities = [self.unfold_kp * e for e in errors]
                # Scale all joints together so the joint-space path stays straight.
                scale = min(
                    1.0, self.unfold_max_speed / max(abs(v) for v in velocities)
                )
                msg.velocities = [v * scale for v in velocities]
                self.joint_pub.publish(msg)
                return

            msg.velocities = [0.0] * len(names)
            self.joint_pub.publish(msg)

            self.get_logger().info('Unfold complete. Switching to Cartesian control.')
            self.startup_phase = 'SWITCHING'
            self.start_twist_controller_switch()
            return

        # ---------------------------------------------------------
        # WAIT FOR CARTESIAN CONTROLLER
        # ---------------------------------------------------------
        if self.startup_phase == 'SWITCHING':

            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = list(self.unfold_target)
            msg.velocities = [0.0] * len(self.unfold_target)
            self.joint_pub.publish(msg)

            if self.switch_future is None:
                return

            if not self.switch_future.done():
                return

            result = self.switch_future.result()

            if result is None or not result.ok:
                self.get_logger().error(
                    'Failed to switch to delta_twist_controller.'
                )
                self.startup_phase = 'FAILED'
                return

            self.get_logger().info(
                'Startup escape complete. Cartesian control starting.'
            )

            self.startup_phase = 'RUNNING'
            return

        # ---------------------------------------------------------
        # FAILED STATE
        # ---------------------------------------------------------
        if self.startup_phase == 'FAILED':
            return

        # ---------------------------------------------------------
        # REQUIRED FEEDBACK
        # ---------------------------------------------------------
        if (
            self.tcp_pose is None
            or self.joint_angles is None
            or self.arm_status is None
        ):
            return

        # ---------------------------------------------------------
        # SAFETY CHECK
        # ---------------------------------------------------------
        if self.arm_status >= 20:

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = base_frame

            # Explicit zero command.
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            self.twist_pub.publish(msg)

            if self.arm_status != self.last_logged_status:
                self.last_logged_status = self.arm_status
                self.get_logger().warn(
                    f'Cartesian motion stopped: arm_status={self.arm_status}'
                )

            return

        # ---------------------------------------------------------
        # CAPTURE INITIAL TOOL ORIENTATION
        # ---------------------------------------------------------
        if self.target_orientation is None:
            self.target_orientation = self.tcp_pose.orientation

        # ---------------------------------------------------------
        # ALL WAYPOINTS COMPLETED
        # ---------------------------------------------------------
        if self.current_waypoint >= len(waypoints):

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = base_frame

            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            self.twist_pub.publish(msg)
            return

        # ---------------------------------------------------------
        # CURRENT TCP POSITION
        # ---------------------------------------------------------
        current_x = self.tcp_pose.position.x
        current_y = self.tcp_pose.position.y
        current_z = self.tcp_pose.position.z

        # ---------------------------------------------------------
        # ACTUAL TARGET SELECTION
        #
        # The target MUST be selected before calculating velocity.
        # ---------------------------------------------------------
        target = waypoints[self.current_waypoint]

        # ---------------------------------------------------------
        # POSITION ERROR
        # ---------------------------------------------------------
        error_x = target[0] - current_x
        error_y = target[1] - current_y
        error_z = target[2] - current_z

        distance = math.sqrt(
            error_x * error_x +
            error_y * error_y +
            error_z * error_z
        )

        # ---------------------------------------------------------
        # WAYPOINT / ROUTE ARRIVAL CHECK
        # ---------------------------------------------------------
        route_tolerance = self.position_tolerance

        if distance <= route_tolerance:

            # Normal waypoint arrival
            if self.hold_start_time is None:

                self.hold_start_time = self.get_clock().now()

                self.get_logger().info(
                    f'Waypoint {self.current_waypoint + 1} reached. '
                    f'Holding for 2.3 seconds.'
                )

            else:

                elapsed = (
                    self.get_clock().now() -
                    self.hold_start_time
                ).nanoseconds / 1e9

                if elapsed >= 2.3:

                    self.get_logger().info(
                        f'Waypoint {self.current_waypoint + 1} hold complete.'
                    )

                    self.current_waypoint += 1
                    self.hold_start_time = None

            # Always publish zero while holding.
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = base_frame

            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            self.twist_pub.publish(msg)
            return

        # We are moving again, so cancel any previous hold timer.
        self.hold_start_time = None

        # ---------------------------------------------------------
        # POSITION PROPORTIONAL CONTROL
        # ---------------------------------------------------------
        vx = self.position_kp * error_x
        vy = self.position_kp * error_y
        vz = self.position_kp * error_z

        linear_speed = math.sqrt(
            vx * vx +
            vy * vy +
            vz * vz
        )

        # Scale the entire vector, never individual axes.
        if linear_speed > cap_linear_mps:

            scale = cap_linear_mps / linear_speed

            vx *= scale
            vy *= scale
            vz *= scale

        # ---------------------------------------------------------
        # ORIENTATION CONTROL
        # ---------------------------------------------------------
        current_rot = Rotation.from_quat([
            self.tcp_pose.orientation.x,
            self.tcp_pose.orientation.y,
            self.tcp_pose.orientation.z,
            self.tcp_pose.orientation.w
        ])

        target_rot = self.tool_target_rotation(target)

        error_rot = target_rot * current_rot.inv()
        rotvec = error_rot.as_rotvec()

        wx = self.orientation_kp * rotvec[0]
        wy = self.orientation_kp * rotvec[1]
        wz = self.orientation_kp * rotvec[2]

        angular_speed = math.sqrt(
            wx * wx +
            wy * wy +
            wz * wz
        )

        if angular_speed > cap_angular_rps:

            scale = cap_angular_rps / angular_speed

            wx *= scale
            wy *= scale
            wz *= scale

        # ---------------------------------------------------------
        # PUBLISH CARTESIAN COMMAND
        # ---------------------------------------------------------
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame

        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz

        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz

        self.twist_pub.publish(msg)
    ##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node

    node.get_logger().info('Node created: Arm waypoints process')   # logging information

    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'

    rclpy.spin(arm_waypoints_class)                                 # spining on the object to make it alive in ROS 2 DDS

    arm_waypoints_class.destroy_node()                              # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()

