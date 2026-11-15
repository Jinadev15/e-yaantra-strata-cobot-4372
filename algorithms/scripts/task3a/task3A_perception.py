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
# Filename:		    task3A_perception.py
# Functions:
#			        main
# Classes:          OreKind, Detection, OreDetector, DepthProjector, TrackedOre, OreTracker,
#                   PackageTracker, ore_perception
# Nodes:		    ore_perception_node
#                   Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                          /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info, /tf_static,
#                                          /handover/ebot, /handover/arm ]


################### IMPORT MODULES #######################

import sys
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


##################### TASK CONSTANTS #######################

color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

base_frame = 'base_link'                      # every item transform is published against this frame
expected_ore_count = 4                        # the task puts four ores on the conveyor
package_frame = 'ore_package'

# The handover states (latched): the package is looked for only while the eBot stands beside
# the arm, and published until the arm reports it lifted off the deck.
ebot_state_topic = '/handover/ebot'
arm_state_topic = '/handover/arm'
ebot_at_arm = 'AT_ARM'
arm_taking = 'TAKING'
arm_package_clear = 'PACKAGE_CLEAR'

# Ore block and ore package (The Magnetic Tool): both 76.2 mm tall. The camera sees the top
# face; an item is named by its middle.
item_height_m = 0.0762

# --- Colour segmentation (HSV, H 0-179). Read off a saved frame of the simulated camera. ---
min_blob_area_px = 80                         # smaller regions are noise
max_blob_area_px = 2000                       # an ore's top face is about 150-550 px; the ground is far larger
dark_ring_fraction = 0.70                     # an ore top face is ringed by black sides, rock and rover are not
dark_value_max = 70                           # HSV value under this counts as black
ring_inner_px = 3                             # the ring around a blob lies between these two dilations
ring_outer_px = 9
open_kernel_px = 3

# --- Depth ---
depth_patch_px = 9                            # square window around a blob centre, median of its valid pixels

# --- Tracking ---
match_radius_m = 0.06                         # a sighting this close to a known ore of its kind is that ore
carry_match_radius_m = 0.25                   # once the set is fixed, a moving (carried) ore is followed this far
history_length = 15                           # sightings averaged into one position
stable_sightings = 10                         # sightings an ore needs before it counts as established
candidate_spare = 4                           # stray detections that may be followed without taking an ore's place
publish_rate_s = 0.1                          # how often ore transforms are broadcast


##################### CLASS DEFINITION #######################

@dataclass(frozen=True)
class OreKind:
    '''
    ___CLASS___

    Description:    One kind of item, the HSV range of its coloured top face, and the share
                    of black a ring round that face must hold.
    '''

    name: str
    lower: tuple
    upper: tuple
    ring_fraction: float = dark_ring_fraction


ore_kinds = (
    OreKind('azurite', (100, 130, 120), (110, 255, 220)),
    OreKind('malachite', (68, 90, 190), (80, 255, 255)),
    OreKind('vanadinite', (7, 110, 150), (13, 255, 255)),
)

# The package's top face is magenta (H 154 in the simulated camera) with a black rim; nothing
# else in view is magenta. Its rim is thinner than an ore's sides and the eBot's white body
# shows beyond it, so the ring holds about 67 % black instead of an ore's 70 % and more.
package_kind = OreKind('package', (145, 150, 150), (163, 255, 255), ring_fraction=0.50)


@dataclass(frozen=True)
class Detection:
    '''
    ___CLASS___

    Description:    One ore found in one colour frame: its kind and the pixel at the middle
                    of its top face.
    '''

    kind: str
    u: int
    v: int


class OreDetector:
    '''
    ___CLASS___

    Description:    Finds the ores in a colour frame. An ore is a coloured top face with black
                    sides, so a coloured region only counts when black surrounds it. That is
                    what separates an orange ore from the orange ground and the rover.
    '''

    def __init__(self, kinds=ore_kinds):
        self.kinds = kinds
        self._open_kernel = np.ones((open_kernel_px, open_kernel_px), np.uint8)
        self._inner = np.ones((ring_inner_px, ring_inner_px), np.uint8)
        self._outer = np.ones((ring_outer_px, ring_outer_px), np.uint8)

    def detect(self, image_bgr):
        '''
        Args:
            image_bgr (ndarray):    colour frame, BGR, shape (rows, cols, 3)

        Returns:
            list of Detection, ordered by kind and then by position in the frame.
        '''

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        black = hsv[..., 2] < dark_value_max
        found = []
        for kind in self.kinds:
            mask = cv2.inRange(hsv, np.array(kind.lower), np.array(kind.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if self._is_item_face(contour, black, kind.ring_fraction):
                    moments = cv2.moments(contour)
                    found.append(Detection(kind.name,
                                           int(moments['m10'] / moments['m00']),
                                           int(moments['m01'] / moments['m00'])))
        return sorted(found, key=lambda d: (d.kind, d.v, d.u))

    def _is_item_face(self, contour, black, ring_fraction):
        '''True for a region of item-face size that black surrounds. Only the crop round the region is examined.'''
        if not min_blob_area_px <= cv2.contourArea(contour) <= max_blob_area_px:
            return False
        x, y, w, h = cv2.boundingRect(contour)
        left, top = max(x - ring_outer_px, 0), max(y - ring_outer_px, 0)
        right = min(x + w + ring_outer_px, black.shape[1])
        bottom = min(y + h + ring_outer_px, black.shape[0])
        blob = np.zeros((bottom - top, right - left), np.uint8)
        cv2.drawContours(blob, [contour], -1, 255, cv2.FILLED, offset=(-left, -top))
        ring = (cv2.dilate(blob, self._outer) > 0) & (cv2.dilate(blob, self._inner) == 0)
        if not ring.any():
            return False
        return bool(black[top:bottom, left:right][ring].mean() >= ring_fraction)


class DepthProjector:
    '''
    ___CLASS___

    Description:    Turns a pixel of the colour image into a point in base_link: depth from the
                    aligned depth image, the pinhole model from camera_info, then the static
                    camera-to-base transform.
    '''

    def __init__(self):
        self.fx = self.fy = self.cx = self.cy = None
        self._rotation = None                     # camera optical frame -> base_link, cached: the transform is static
        self._translation = None

    @property
    def has_intrinsics(self):
        return self.fx is not None

    @property
    def has_extrinsics(self):
        return self._rotation is not None

    def set_intrinsics(self, camera_info):
        '''Read the pinhole matrix from the topic; it is never hard-coded.'''
        self.fx, self.fy = camera_info.k[0], camera_info.k[4]
        self.cx, self.cy = camera_info.k[2], camera_info.k[5]

    def set_extrinsics(self, transform):
        '''Cache a geometry_msgs Transform from the camera optical frame to base_link.'''
        q, t = transform.rotation, transform.translation
        self._rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        self._translation = np.array([t.x, t.y, t.z])

    def project(self, u, v, depth_image):
        '''
        Returns:
            The point (x, y, z) in base_link at pixel (u, v), or None when the depth patch
            around it holds no valid reading.
        '''

        half = depth_patch_px // 2
        patch = depth_image[max(v - half, 0):v + half + 1, max(u - half, 0):u + half + 1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]     # NaN and inf come back from out-of-range surfaces
        if valid.size == 0:
            return None
        z = float(np.median(valid))                            # median: edge pixels flicker between surfaces
        in_camera = np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z])
        return self._rotation.apply(in_camera) + self._translation


@dataclass
class TrackedOre:
    '''
    ___CLASS___

    Description:    One ore followed over time: its kind, the recent sightings of its middle and,
                    once it is established, its name.
    '''

    kind: str
    name: str = ''
    sightings: deque = field(default_factory=lambda: deque(maxlen=history_length))

    @property
    def position(self):
        '''Median of the recent sightings, which shrugs off a single bad depth reading.'''
        return np.median(np.asarray(self.sightings), axis=0)

    @property
    def established(self):
        return len(self.sightings) >= stable_sightings

    def distance_to(self, point):
        return float(np.linalg.norm(self.position - point))


class OreTracker:
    '''
    ___CLASS___

    Description:    Gives every ore a name that lasts the whole run. It first collects the
                    ores while all of them are in view; a stray detection that never repeats
                    stays a candidate and is never published. Once the expected number of ores
                    is established the set is fixed: a later sighting only moves the nearest
                    ore of its kind, so an ore being carried keeps its name and no new one appears.
    '''

    def __init__(self, expected_count=expected_ore_count):
        self.expected_count = expected_count
        self.ores = []
        self._next_number = {}

    @property
    def is_fixed(self):
        return len(self.established_ores()) >= self.expected_count

    def update(self, kind, point):
        '''Fold one sighting (kind, base_link point) into the tracked ores.'''
        radius = carry_match_radius_m if self.is_fixed else match_radius_m
        candidates = [o for o in self.ores if o.kind == kind]
        nearest = min(candidates, key=lambda o: o.distance_to(point), default=None)
        if nearest is not None and nearest.distance_to(point) <= radius:
            nearest.sightings.append(point)
            self._name_if_established(nearest)
        elif not self.is_fixed and len(self.ores) < self.expected_count + candidate_spare:
            ore = TrackedOre(kind)
            ore.sightings.append(point)
            self.ores.append(ore)

    def _name_if_established(self, ore):
        '''Names count up from 1 per kind in the order ores become established, so a stray never takes a number.'''
        if ore.established and not ore.name:
            number = self._next_number.get(ore.kind, 0) + 1
            self._next_number[ore.kind] = number
            ore.name = f'{ore.kind}_ore_{number}'

    def established_ores(self):
        return [o for o in self.ores if o.established]


class PackageTracker:
    '''
    ___CLASS___

    Description:    Follows the one ore package while it stands on the eBot beside the arm.
                    Its position is the median of the recent sightings; it is published once
                    established, and frozen while the arm takes it, when the tool covers it.
    '''

    def __init__(self):
        self.package = TrackedOre(package_kind.name, package_frame)

    @property
    def established(self):
        return self.package.established

    @property
    def position(self):
        return self.package.position

    def update(self, point):
        '''Fold one sighting in; a sighting far from an established package is not the package.'''
        if self.package.established and self.package.distance_to(point) > match_radius_m:
            return
        self.package.sightings.append(point)

    def reset(self):
        '''Forget the package: a new arrival is tracked from scratch.'''
        self.package.sightings.clear()


class ore_perception(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores on the conveyor and the
                    ore package on the eBot beside the arm, and broadcast a transform, in
                    base_link, for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_perception
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ore_perception_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        self.color_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.ebot_state_sub = self.create_subscription(String, ebot_state_topic, self.ebotstatecb, latched)
        self.arm_state_sub = self.create_subscription(String, arm_state_topic, self.armstatecb, latched)

        ############ Constructor VARIABLES/OBJECTS ############

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.detector = OreDetector()
        self.projector = DepthProjector()
        self.tracker = OreTracker()
        self.package_detector = OreDetector((package_kind,))
        self.package_tracker = PackageTracker()
        self.ebot_state = ''
        self.arm_state = ''
        self.announced_package = False
        self.color_image = None
        self.depth_image = None
        self.new_frame = False                    # a colour frame not yet examined
        self.camera_frame = None
        self.announced_fixed = False
        self.timer = self.create_timer(publish_rate_s, self.process_image)

    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.

        Args:
            data (Image):    Input coloured raw image frame received from the camera topic
        '''

        self.color_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.new_frame = True

    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic. The image is
                        32FC1, one float32 metre value per pixel.

        Args:
            data (Image):    Input depth image frame received from the aligned depth topic
        '''

        image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.depth_image = image * 0.001 if data.encoding == '16UC1' else image       # 16UC1 is millimetres

    def ebotstatecb(self, data):
        '''
        Description:    Callback function for the eBot's handover state.

        Args:
            data (String):    State published by the planning node
        '''

        if data.data == ebot_at_arm and self.ebot_state != ebot_at_arm:
            self.package_tracker.reset()
            self.announced_package = False
        self.ebot_state = data.data

    def armstatecb(self, data):
        '''
        Description:    Callback function for the arm's handover state.

        Args:
            data (String):    State published by the manipulation node
        '''

        self.arm_state = data.data

    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.

        Args:
            data (CameraInfo):    Camera calibration published by the camera
        '''

        self.projector.set_intrinsics(data)
        self.camera_frame = data.header.frame_id

    def process_image(self):
        '''
        Description:    Timer function: detect the ores in the latest frame, update the
                        tracker and broadcast a transform for every established ore.
        '''

        if self.color_image is None or self.depth_image is None or not self.projector.has_intrinsics:
            return
        if not self.projector.has_extrinsics and not self.load_camera_transform():
            return

        watching_package = self.ebot_state == ebot_at_arm and self.arm_state != arm_package_clear
        if self.new_frame:                                                             # the timer is faster than the camera
            self.new_frame = False
            for detection in self.detector.detect(self.color_image):
                point = self.item_middle(detection)
                if point is not None:
                    self.tracker.update(detection.kind, point)
            if watching_package and self.arm_state != arm_taking:
                for detection in self.package_detector.detect(self.color_image):
                    point = self.item_middle(detection)
                    if point is not None:
                        self.package_tracker.update(point)

        stamp = self.get_clock().now().to_msg()
        for ore in self.tracker.established_ores():
            self.broadcaster.sendTransform(self.ore_transform(ore, stamp))
        if watching_package and self.package_tracker.established:
            self.broadcaster.sendTransform(self.ore_transform(self.package_tracker.package, stamp))
            if not self.announced_package:
                self.announced_package = True
                x, y, z = self.package_tracker.position
                self.get_logger().info(f'Ore package on the eBot at ({x:.3f}, {y:.3f}, {z:.3f}) in {base_frame}')

        if self.tracker.is_fixed and not self.announced_fixed:
            self.announced_fixed = True
            summary = ', '.join(
                f'{o.name} ({o.position[0]:.3f}, {o.position[1]:.3f}, {o.position[2]:.3f})'
                for o in self.tracker.established_ores())
            self.get_logger().info(f'Ores fixed in {base_frame}: {summary}')

    def item_middle(self, detection):
        '''The middle of a detected item in base_link: its top face, lowered by half its height.'''
        point = self.projector.project(detection.u, detection.v, self.depth_image)
        if point is not None:
            point[2] -= item_height_m / 2.0
        return point

    def load_camera_transform(self):
        '''Look the static camera-to-base transform up once. Returns False until the tree has it.'''
        try:
            stamped = self.tf_buffer.lookup_transform(base_frame, self.camera_frame, rclpy.time.Time())
        except Exception:
            return False
        self.projector.set_extrinsics(stamped.transform)
        return True

    @staticmethod
    def ore_transform(ore, stamp):
        '''The transform base_link -> name for a tracked item.'''
        x, y, z = (float(c) for c in ore.position)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = base_frame
        transform.child_frame_id = ore.name
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        return transform


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ore_perception class to perform its task
    '''

    rclpy.init(args=sys.argv)                                                           # initialisation

    ore_perception_class = ore_perception()                                             # creating a new object for class 'ore_perception'

    try:
        rclpy.spin(ore_perception_class)                                                # spinning on the object to make it alive in ROS 2 DDS
    except KeyboardInterrupt:
        pass
    finally:
        ore_perception_class.destroy_node()                                             # destroy node after spin ends
        if rclpy.ok():
            rclpy.shutdown()                                                            # shutdown process


if __name__ == '__main__':

    main()
