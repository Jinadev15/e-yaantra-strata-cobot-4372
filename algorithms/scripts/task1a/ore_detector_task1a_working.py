#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task1a_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import CameraInfo, Image


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    center_ore_list = []
    ore_type_list = []

    ############ ADD YOUR CODE HERE ############
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_blue = np.array([100, 130, 120])
    upper_blue = np.array([110, 255, 220])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)	
    lower_green = np.array([68, 90, 190])
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    lower_orange = np.array([7, 110, 150])
    upper_orange = np.array([13, 255, 255])
    orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)
    # INSTRUCTIONS & HELP :

    #	->  Detect the ores by COLOUR, and return a center pixel and a type for each.
    #       ->  HINT: hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #                 mask = cv2.inRange(hsv, lower, upper)       # one pair per type
    #                 H is 0-179, S and V are 0-255. The frame is BGR, not RGB.

    #   ->  Read the bounds off a saved frame rather than copying them from a tutorial.

    #   ->  Clean the mask, and drop anything too small to be an ore.
    #       ->  HINT: cv2.morphologyEx (MORPH_OPEN, MORPH_CLOSE), cv2.contourArea

    #   ->  Reduce each region to one center pixel.
    #       ->  HINT: M  = cv2.moments(contour)                   # guard against m00 == 0
    #                 cX = int(M['m10'] / M['m00'])
    #                 cY = int(M['m01'] / M['m00'])
    #   ->  Draw your detections on the frame while you are developing.
    #       ->  HINT: cv2.circle, cv2.putText

    ############################################
    masks = [
        (blue_mask, "azurite"),
        (green_mask, "malachite"),
        (orange_mask, "vanadinite")
    ]

    for mask, ore_type in masks:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            

            if area < 100:
                continue

            if area > 5000:
                continue
            

            M = cv2.moments(contour)

            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if ore_type == "vanadinite" and cx < 470:
                continue

            center_ore_list.append((cx, cy))
            ore_type_list.append(ore_type)
            

    return center_ore_list, ore_type_list

def get_ore_id(self, ore_type, center):
    positions = self.ore_positions[ore_type]

    for i, old_center in enumerate(positions):
        distance = math.sqrt(
            (center[0] - old_center[0]) ** 2 +
            (center[1] - old_center[1]) ** 2
        )

        if distance < 50:
            positions[i] = center
            return i + 1

    if len(positions) < 2:
        positions.append(center)
        return len(positions)

    return None
##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None                                                            # camera intrinsics variable (from caminfocb())
        self.ore_positions = {
          "azurite": [],
          "malachite": [],
          "vanadinite": []
        }
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your detection needs to keep between frames.
        #       ->  HINT: The two ores of a type must keep their ids for the whole run, and
        #                 'detect_ores' returns them unordered.

        ############################################


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data,
            desired_encoding='passthrough'
        )

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        #   ->  Get the units right. Print 'depth.dtype'.
        #       ->  HINT: 32FC1 is METRES; 16UC1 is MILLIMETRES, so divide by 1000.

        #   ->  Drop the pixels that carry no reading.
        #       ->  HINT: depth[np.isfinite(depth) & (depth > 0.0)]

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(
            data,
            desired_encoding='bgr8'
        )

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        ############################################


    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cam_info = data

        self.fx = data.k[0]
        self.fy = data.k[4]
        self.cx = data.k[2]
        self.cy = data.k[5]

        # INSTRUCTIONS & HELP :

        #	->  Store the focal lengths and the principal point. Read them from the topic;
        #       never hard-code them.
        #       ->  HINT: 'k' is the pinhole matrix flattened row by row-
        #                     k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        if self.cv_image is None or self.depth_image is None or self.cam_info is None:
            return
        center_ore_list, ore_type_list = detect_ores(self.cv_image)
        self.get_logger().info(
            f"Detected ores: {len(center_ore_list)} | Types: {ore_type_list}"
        )
        for (cX, cY) in center_ore_list:
            patch = self.depth_image[cY-4:cY+5, cX-4:cX+5]
            valid_depth = patch[np.isfinite(patch) & (patch > 0.0)]

            if valid_depth.size == 0:
                continue

            z = np.median(valid_depth)
            x = (cX - self.cx) * z / self.fx
            y = (cY - self.cy) * z / self.fy

            point_in_camera = PointStamped()
            point_in_camera.header.frame_id = self.cam_info.header.frame_id
            point_in_camera.header.stamp = self.get_clock().now().to_msg()

            point_in_camera.point.x = float(x)
            point_in_camera.point.y = float(y)
            point_in_camera.point.z = float(z)

            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link',
                    point_in_camera.header.frame_id,
                    rclpy.time.Time()
                )

                point_in_base = do_transform_point(point_in_camera, tf)
                ore_type = ore_type_list[center_ore_list.index((cX, cY))]
                ore_id = get_ore_id(self, ore_type, (cX, cY))

                if ore_id is None:
                    continue
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'base_link'
                t.child_frame_id = f'{ore_type}_ore_{ore_id}'

                t.transform.translation.x = point_in_base.point.x
                t.transform.translation.y = point_in_base.point.y
                t.transform.translation.z = point_in_base.point.z

                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.br.sendTransform(t)
            except Exception as e:
                continue
        # INSTRUCTIONS & HELP :

        #	->  Return early until both images and the camera info have arrived.

        #   ->  Get the ore centers and their types from 'detect_ores' defined above

        #   ->  Read the depth at each center pixel. Depth is ALIGNED to colour.
        #       ->  HINT: Take the MEDIAN of a small window, not the one pixel-
        #                     patch = self.depth_image[cY-4:cY+5, cX-4:cX+5]
        #                     z     = np.median(patch[np.isfinite(patch) & (patch > 0.0)])

        #   ->  Deproject the center pixel (u, v) and its depth z into a 3D point-
        #           x = (u - cx) * z / fx
        #           y = (v - cy) * z / fy
        #           z = z
        #       ->  HINT: That point is in the camera's OPTICAL frame - 'data.header.frame_id'.

        #   ->  Transform it into 'base_frame'-
        #           tf = self.tf_buffer.lookup_transform(
        #                    base_frame, <optical frame>, rclpy.time.Time())
        #           p  = do_transform_point(point_in_camera, tf)
        #       ->  HINT: PointStamped and tf2_geometry_msgs are not imported above, and the
        #                 lookup raises until the tree has filled in.

        #   ->  Correct for what you measured: the camera sees the ore's top face, and the
        #       ore is named by its middle.

        #   ->  Give each ore an id and keep it for the whole run.
        #       ->  HINT: Match a detection to the nearest ore of that type already named,
        #                 on the pixel rather than the depth.

        #   ->  Publish one transform per ore, using Geometry Message - TransformStamped
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = '<ore_type>_<id>'      Ex: azurite_ore_1, where azurite_ore
        #                                                   is one of 'ore_types' and the id is 1 or 2
        #       ->  HINT: t.header.stamp, t.header.frame_id, t.child_frame_id,
        #                 t.transform.translation.x/.y/.z, t.transform.rotation.w = 1.0,
        #                 then self.br.sendTransform(t). Every number must be a float.
        #       ->  NOTE: Only the translation is read; the names are matched exactly.

        #   ->  Broadcast every ore on every cycle, not once.

        #   ->  Show the frame with your detections drawn on it using 'cv2.imshow'.

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the ore_tf
                    class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ore_tf_process')                      # creating ROS node

    node.get_logger().info('Node created: Ore tf process')          # logging information

    ore_tf_class = ore_tf()                                         # creating a new object for class 'ore_tf'

    rclpy.spin(ore_tf_class)                                        # spining on the object to make it alive in ROS 2 DDS

    ore_tf_class.destroy_node()                                     # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
