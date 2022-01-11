#!/usr/bin/env python3


# stdlib
import os

import atom_core.ros_utils
import math
import random
import threading
import time

# from __builtin__ import enumerate
from copy import deepcopy

# 3rd-party
import colorama
import cv2
import rospy
import numpy as np
import ros_numpy
import scipy.spatial

import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge
from matplotlib import cm
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from rospy_message_converter import message_converter
from sensor_msgs.msg import *
from sensor_msgs.msg import PointField
from image_geometry import PinholeCameraModel
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

import numpy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import MarkerArray
import random

import math
import image_geometry

# local packages
from atom_calibration.collect import patterns
import atom_core.utilities
from atom_calibration.collect.label_messages import labelPointCloud2Msg, labelDepthMsg2, calculateFrustrum, \
    pixToWorld, worldToPix

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
                [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8


def createRosCloud(points, stamp, frame_id, colours=None):
    # Set header
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points = np.asarray(points)
    if not colours:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        tmpColours = np.floor(np.asarray(colours) * 255)  # nx3 matrix
        tmpColours = tmpColours[:, 0] * BIT_MOVE_16 + tmpColours[:, 1] * BIT_MOVE_8 + tmpColours[:, 2]
        cloud_data = np.c_[points, tmpColours]

    # Create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


# ------------------------
#      BASE CLASSES      #
# ------------------------

# return Fore.GREEN + self.parent + Style.RESET_ALL + ' to ' + Fore.GREEN + self.child + Style.RESET_ALL + ' (' +
# self.joint_type + ')'

class LaserScanCluster:
    """
    An auxiliary class for storing information about 2D laser clusters
    """

    def __init__(self, cluster_count, idx):
        self.cluster_count = cluster_count
        self.idxs = [idx]

    def pushIdx(self, idx):
        self.idxs.append(idx)

    def __str__(self):
        return "Cluster " + str(self.cluster_count) + " contains idxs = " + str(self.idxs)


class InteractiveDataLabeler:
    """
    Handles data labelling for a generic sensor:
        Cameras: Fully automated labelling. Periodically runs a chessboard detection on the newly received image.
        LaserScans: Semi-automated labelling. An rviz interactive marker is placed on the laser cluster which contains
                    the calibration pattern, and the pattern is tracked from there onward.
        PointCloud2: #TODO Tiago Madeira can you complete?
    """

    def __init__(self, server, menu_handler, sensor_dict, marker_scale, calib_pattern, label_data=True):
        """
        Class constructor. Initializes several variables and ros stuff.
        :param server: an interactive marker server
        :param menu_handler: an interactive MenuHandler
        :param sensor_dict: A dictionary that describes the sensor
        :param marker_scale: scale of the markers to be drawn
        :param chess_numx: chessboard size in x
        :param chess_numy: chessboard size in y
        """
        print('Creating an InteractiveDataLabeler for sensor ' + str(sensor_dict['_name']))

        # Store variables to class attributes
        self.label_data = label_data
        self.menu_handler = menu_handler
        self.name = sensor_dict['_name']
        self.parent = sensor_dict['parent']
        self.topic = sensor_dict['topic']
        self.modality = sensor_dict['modality']
        self.marker_scale = marker_scale
        self.received_first_msg = False
        self.labels = {'detected': False, 'idxs': []}
        # self.server = server
        self.server = InteractiveMarkerServer(self.name + "/data_labeler")
        self.lock = threading.Lock()
        self.seed = {'x': 510, 'y': 280}
        self.marker_array = MarkerArray()

        # self.calib_pattern = calib_pattern
        if calib_pattern['pattern_type'] == 'chessboard':
            self.pattern = patterns.ChessboardPattern(calib_pattern['dimension'], calib_pattern['size'])
        elif calib_pattern['pattern_type'] == 'charuco':
            self.pattern = patterns.CharucoPattern(calib_pattern['dimension'], calib_pattern['size'],
                                                   calib_pattern['inner_size'], calib_pattern['dictionary'])
            print(calib_pattern['dictionary'])
        else:
            print("Unknown pattern type '{}'".format(calib_pattern['pattern_type']))
            sys.exit(1)

        # Get the type of message from the message topic of the sensor data, which is given as input. The message
        # type is used to define which labelling technique is used.
        self.msg_type_str, self.msg_type = atom_core.ros_utils.getMessageTypeFromTopic(self.topic)
        print('msg_type_str is = ' + str(self.msg_type_str))

        # TODO decide which labeler to use

        # Handle the interactive labelling of data differently according to the sensor message types.
        # if self.msg_type_str in ['LaserScan'] and self.label_data:
        if self.modality == 'lidar2d' and self.label_data:
            # TODO parameters given from a command line input?
            self.threshold = 0.2  # pt to pt distance  to create new cluster (param  only for 2D LIDAR labelling)
            self.minimum_range_value = 0.3  # distance to assume range value valid (param only for 2D LIDAR labelling)
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled',
                                                             sensor_msgs.msg.PointCloud2,
                                                             queue_size=1)  # publish a point cloud with the points
            # in the selected cluster
            self.publisher_clusters = rospy.Publisher(self.topic + '/clusters', sensor_msgs.msg.PointCloud2,
                                                      queue_size=1)  # publish a point cloud with coloured clusters
            self.createInteractiveMarker()  # interactive marker to label the calibration pattern cluster (one time)
            print('Created interactive marker for laser scans.')
        # elif self.msg_type_str in ['Image'] and self.label_data:
        elif self.modality == 'rgb' and self.label_data:
            self.bridge = CvBridge()  # a CvBridge structure is needed to convert opencv images to ros messages.
            self.publisher_labelled_image = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.Image,
                                                            queue_size=1)  # publish
            # images with the detected chessboard overlaid onto the image.

        # elif self.msg_type_str in ['PointCloud2'] and self.label_data:  # Velodyne data (Andre Aguiar)
        elif self.modality == 'lidar3d' and self.label_data:  # Velodyne data (Andre Aguiar)
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.PointCloud2,
                                                             queue_size=1)  # publish a point cloud with the points
            self.createInteractiveMarkerRGBD(x=0.804, y=0.298,
                                             z=0.409)  # interactive marker to label the calibration pattern
            # cluster (one time)

            # Labeler definitions
            # Hessian plane coefficients
            self.A = 0
            self.B = 0
            self.C = 0
            self.D = 0
            self.number_iterations = 15  # RANSAC number of iterations
            self.ransac_threshold = 0.01  # RANSAC point-to-plane distance threshold to consider inliers
            # Chessboard point tracker distance threshold
            self.tracker_threshold = math.sqrt(((calib_pattern['dimension']['x'] - 1) * calib_pattern['size']) ** 2 + \
                                               ((calib_pattern['dimension']['y'] - 1) * calib_pattern[
                                                   'size']) ** 2) * 0.8

            print('Created interactive marker for point clouds.')

        elif self.modality == 'depth' and self.label_data:  # Velodyne data (Andre Aguiar)
            # print('Depth labeller under construction')
            self.bridge = CvBridge()  # a CvBridge structure is needed to convert opencv images to ros messages.
            self.publisher_labelled_depth = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.Image,
                                                            queue_size=1)  # publish
            # self.publisher_frustrum = rospy.Publisher(self.topic + '/frustum', Marker, queue_size=1)
            self.pinhole_camera_model = image_geometry.PinholeCameraModel()
            self.pinhole_camera_model.fromCameraInfo(
                message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                    sensor_dict['camera_info']))
            self.createInteractiveMarkerRGBD(x=0.804, y=0.298,
                                             z=0.409)
            # self.camera_info = rospy.wait_for_message(self.topic, CameraInfo)



        else:
            if self.label_data:
                # We handle only know message types
                raise ValueError(
                    'Message type ' + self.modality + ' for topic ' + self.topic + 'is of an unknown type.')
                # self.publisher = rospy.Publisher(self.topic + '/labeled', self.msg_type, queue_size=0)

        # Subscribe to the message topic containing sensor data
        # https://github.com/ros/ros_comm/issues/536
        # https://github.com/lmb-freiburg/rgbd-pose3d/issues/5
        # TODO figure out which is the adequate size, this was trial and error
        # self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self.sensorDataReceivedCallback, queue_size=1, buff_size=10000000000)
        self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self.sensorDataReceivedCallback, queue_size=1,
                                           buff_size=10000000000)
        self.publisher_frustrum=rospy.Publisher('sensor_frustum', MarkerArray, queue_size=1)

    def sensorDataReceivedCallback(self, msg):
        # rospy.loginfo(self.name + ' (Before lock) received msg which is ' + str((rospy.Time.now() - msg.header.stamp).to_sec()) + ' secs.')

        stamp_before_lock = rospy.Time.now()
        self.lock.acquire()  # use semaphores to make sure the data is not being written on two sides simultaneously
        # self.lock.acquire(blocking=False)  # use semaphores to make sure the data is not being written on two sides simultaneously
        self.msg = msg  # make a local copy of sensor data
        now = rospy.Time.now()
        if self.label_data:
            self.labelData  # label the data
            self.publisher_frustrum.publish(self.marker_array)
        # rospy.loginfo('Labelling data for ' + self.name + ' took ' + str((rospy.Time.now() - now).to_sec()) + ' secs.')
        self.lock.release()  # release lock
        # rospy.loginfo('(With Lock) Labelling data for ' + self.name + ' took ' + str((rospy.Time.now() - stamp_before_lock).to_sec()) + ' secs.')

    @property
    def labelData(self):
        # print('Labelling data for sensor ' + self.name)
        # Reset detected and idxs values to make sure we are not using information from a previous labelling
        self.labels['detected'] = False
        self.labels['idxs'] = []

        # Labelling process dependent of the sensor type
        # if self.msg_type_str == 'LaserScan':  # 2D LIDARS -------------------------------------
        if self.modality == 'lidar2d':  # 2D LIDARS -------------------------------------
            # For 2D LIDARS the process is the following: First cluster all the range data into clusters. Then,
            # associate one of the clusters with the calibration pattern by selecting the cluster which is closest to
            # the rviz interactive marker.

            clusters = []  # initialize cluster list to empty
            cluster_counter = 0  # init counter
            points = []  # init points

            # Compute cartesian coordinates
            xs, ys = atom_core.utilities.laser_scan_msg_to_xy(self.msg)

            # Clustering:
            first_iteration = True
            for idx, r in enumerate(self.msg.ranges):
                # Skip if either this point or the previous have range smaller than minimum_range_value
                if r < self.minimum_range_value or self.msg.ranges[idx - 1] < self.minimum_range_value:
                    continue

                if first_iteration:  # if first iteration, create a new cluster
                    clusters.append(LaserScanCluster(cluster_counter, idx))
                    first_iteration = False
                else:  # check if new point belongs to current cluster, create new cluster if not
                    x = xs[clusters[-1].idxs[-1]]  # x coordinate of last point of last cluster
                    y = ys[clusters[-1].idxs[-1]]  # y coordinate of last point of last cluster
                    distance = math.sqrt((xs[idx] - x) ** 2 + (ys[idx] - y) ** 2)
                    if distance > self.threshold:  # if distance larger than threshold, create new cluster
                        cluster_counter += 1
                        clusters.append(LaserScanCluster(cluster_counter, idx))
                    else:  # same cluster, push this point into the same cluster
                        clusters[-1].pushIdx(idx)

            # Association stage: find out which cluster is closer to the marker
            x_marker, y_marker = self.marker.pose.position.x, self.marker.pose.position.y  # interactive marker pose
            idx_closest_cluster = 0
            min_dist = sys.maxint
            for cluster_idx, cluster in enumerate(clusters):  # cycle all clusters
                for idx in cluster.idxs:  # cycle each point in the cluster
                    x, y = xs[idx], ys[idx]
                    dist = math.sqrt((x_marker - x) ** 2 + (y_marker - y) ** 2)
                    if dist < min_dist:
                        idx_closest_cluster = cluster_idx
                        min_dist = dist

            closest_cluster = clusters[idx_closest_cluster]

            # Find the coordinate of the middle point in the closest cluster and bring the marker to that point
            x_sum, y_sum = 0, 0
            for idx in closest_cluster.idxs:
                x_sum += xs[idx]
                y_sum += ys[idx]

            self.marker.pose.position.x = x_sum / float(len(closest_cluster.idxs))
            self.marker.pose.position.y = y_sum / float(len(closest_cluster.idxs))
            self.marker.pose.position.z = 0
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            # Update the dictionary with the labels
            self.labels['detected'] = True

            percentage_points_to_remove = 0.0  # remove x% of data from each side
            number_of_idxs = len(clusters[idx_closest_cluster].idxs)
            idxs_to_remove = int(percentage_points_to_remove * float(number_of_idxs))
            clusters[idx_closest_cluster].idxs_filtered = clusters[idx_closest_cluster].idxs[
                                                          idxs_to_remove:number_of_idxs - idxs_to_remove]

            self.labels['idxs'] = clusters[idx_closest_cluster].idxs_filtered

            # Create and publish point cloud message with the colored clusters (just for debugging)
            cmap = cm.prism(np.linspace(0, 1, len(clusters)))
            points = []
            z, a = 0, 255
            for cluster in clusters:
                for idx in cluster.idxs:
                    x, y = xs[idx], ys[idx]
                    r, g, b = int(cmap[cluster.cluster_count, 0] * 255.0), \
                              int(cmap[cluster.cluster_count, 1] * 255.0), \
                              int(cmap[cluster.cluster_count, 2] * 255.0)
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt = [x, y, z, rgb]
                    points.append(pt)

            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
            header = Header()
            header.frame_id = self.parent
            header.stamp = self.msg.header.stamp
            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.publisher_clusters.publish(pc_msg)

            # Create and publish point cloud message containing only the selected calibration pattern points
            points = []
            for idx in clusters[idx_closest_cluster].idxs_filtered:
                x_marker, y_marker, z_marker = xs[idx], ys[idx], 0
                r = int(0 * 255.0)
                g = int(0 * 255.0)
                b = int(1 * 255.0)
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x_marker, y_marker, z_marker, rgb]
                points.append(pt)

            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.publisher_selected_points.publish(pc_msg)

        # elif self.msg_type_str == 'Image':  # Cameras -------------------------------------------
        elif self.modality == 'rgb':
            # rospy.loginfo(
            #     'Labelling image for ' + self.name + ' which is ' + str((rospy.Time.now() - self.msg.header.stamp).to_sec()) + ' secs old.')

            # Convert to opencv image and save image to disk
            image = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")
            result = self.pattern.detect(image, equalize_histogram=True)

            if result['detected']:
                c = []

                if 'ids' in result:
                    # The charuco pattern also return an ID for each keypoint.
                    # We can use this information for partial detections.
                    for idx, corner in enumerate(result['keypoints']):
                        c.append({'x': float(corner[0][0]), 'y': float(corner[0][1]), 'id': result['ids'][idx]})
                else:
                    for corner in result['keypoints']:
                        c.append({'x': float(corner[0][0]), 'y': float(corner[0][1])})

                x = int(round(c[0]['x']))
                y = int(round(c[0]['y']))
                cv2.line(image, (x, y), (x, y), (0, 255, 255), 20)

                # Update the dictionary with the labels
                self.labels['detected'] = True
                self.labels['idxs'] = c

            # For visual debugging
            self.pattern.drawKeypoints(image, result)

            msg_out = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg_out.header.stamp = self.msg.header.stamp
            msg_out.header.frame_id = self.msg.header.frame_id
            self.publisher_labelled_image.publish(msg_out)

        # elif self.msg_type_str == 'PointCloud2':  # 3D scan point cloud (Andre Aguiar) ---------------------------------
        elif self.modality == 'lidar3d':  # 3D scan point cloud (Andre Aguiar) ---------------------------------

            # rospy.loginfo(
            #     'Labelling PointCloud for ' + self.name + ' which is ' + str((rospy.Time.now() - self.msg.header.stamp).to_sec()) + ' secs old.')

            # Get the marker position (this comes from the sphere in rviz)
            x_marker, y_marker, z_marker = self.marker.pose.position.x, self.marker.pose.position.y, \
                                           self.marker.pose.position.z  # interactive marker pose

            # Extract 3D point from the ros msg
            self.labels, seed_point, inliers = labelPointCloud2Msg(self.msg, x_marker, y_marker, z_marker,
                                                                   self.tracker_threshold, self.number_iterations,
                                                                   self.ransac_threshold)

            # publish the points that belong to the cluster
            points = []
            for i in range(len(inliers)):
                r = int(1 * 255.0)
                g = int(1 * 255.0)
                b = int(1 * 255.0)
                a = 150
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [inliers[i, 0], inliers[i, 1], inliers[i, 2], rgb]
                points.append(pt)

            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
            header = Header()
            header.frame_id = self.parent
            header.stamp = self.msg.header.stamp
            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.publisher_selected_points.publish(pc_msg)

            # Update the interactive marker pose
            self.marker.pose.position.x = seed_point[0]
            self.marker.pose.position.y = seed_point[1]
            self.marker.pose.position.z = seed_point[2]
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            # print(colorama.Fore.RED + 'Labelled point cloud ' + colorama.Style.RESET_ALL)
            # print(colorama.Fore.RED + 'Aborting ' + colorama.Style.RESET_ALL)
            # exit(0)
        elif self.modality == 'depth':  # depth camera - Daniela ---------------------------------

            # publishing frustrum
            f_x = self.pinhole_camera_model.fx()
            f_y = self.pinhole_camera_model.fy()
            frame_id = self.msg.header.frame_id
            size = self.pinhole_camera_model.fullResolution()
            w = size[0]
            h = size[1]
            marker_frustrum = calculateFrustrum(w, h, f_x, f_y, Z_near=0.3, Z_far=5, frame_id=frame_id, ns=self.topic)
            # self.publisher_frustrum.publish(marker_frustrum)
            self.marker_array.markers.append(marker_frustrum)

            # get seed point from interactive marker
            c_x = self.pinhole_camera_model.cx()
            c_y = self.pinhole_camera_model.cy()
            x_marker, y_marker, z_marker = self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z  # interactive marker pose
            x_pix, y_pix = worldToPix(f_x, f_y, c_x, c_y, x_marker, y_marker, z_marker)
            x_pix = int(x_pix)
            y_pix = int(y_pix)
            # print(self.pinhole_camera_model.fullResolution())

            if 0 < x_pix < self.pinhole_camera_model.fullResolution()[0] and 0 < y_pix < \
                    self.pinhole_camera_model.fullResolution()[1]:
                self.seed['x'] = x_pix
                self.seed['y'] = y_pix

            # actual labelling
            self.labels, result_image, new_seed_point = labelDepthMsg2(self.msg, seed=self.seed,
                                                                       bridge=self.bridge,
                                                                       pyrdown=1, scatter_seed=True, debug=False,
                                                                       subsample_solid_points=6)

            w = self.pinhole_camera_model.fullResolution()[0]
            h = self.pinhole_camera_model.fullResolution()[1]

            # print(x_pix, y_pix)
            if 0 < x_pix < w and 0 < y_pix < h:
                cv2.line(result_image, (x_pix, y_pix), (x_pix, y_pix), (0, 255, 255), 5)
            else:
                x = int(w / 2)
                y = int(h / 2)
                cv2.line(result_image, (x, y), (x, y), (0, 0, 255), 5)

            self.seed['x'] = new_seed_point['x']
            self.seed['y'] = new_seed_point['y']
            # print(result_image.dtype)

            msg_out = self.bridge.cv2_to_imgmsg(result_image, encoding="passthrough")
            msg_out.header.stamp = self.msg.header.stamp
            msg_out.header.frame_id = self.msg.header.frame_id
            self.publisher_labelled_depth.publish(msg_out)

            # update interactive marker
            X, Y, Z = pixToWorld(f_x, f_y, c_x, c_y, self.seed['x'], self.seed['y'], z_marker)

            self.marker.pose.position.x = X
            self.marker.pose.position.y = Y
            self.marker.pose.position.z = Z
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

    def markerFeedback(self, feedback):
        print(' sensor ' + self.name + ' received feedback')

        # pass
        # self.optT.setTranslationFromPosePosition(feedback.pose.position)
        # self.optT.setQuaternionFromPoseQuaternion(feedback.pose.orientation)

        # self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def createInteractiveMarker(self):
        self.marker = InteractiveMarker()
        self.marker.header.frame_id = self.parent
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale = self.marker_scale

        self.marker.name = self.name
        self.marker.description = self.name + '_labeler'

        # insert a box
        control = InteractiveMarkerControl()
        control.always_visible = True

        marker_box = Marker()
        marker_box.type = Marker.SPHERE
        marker_box.scale.x = self.marker.scale * 0.3
        marker_box.scale.y = self.marker.scale * 0.3
        marker_box.scale.z = self.marker.scale * 0.3
        marker_box.color.r = 0
        marker_box.color.g = 1
        marker_box.color.b = 0
        marker_box.color.a = 0.2

        control.markers.append(marker_box)
        self.marker.controls.append(control)

        self.marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 1
        # control.orientation.z = 0
        # control.name = "move_z"
        # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # control.orientation_mode = InteractiveMarkerControl.FIXED
        # self.marker.controls.append(control)
        # #
        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 0
        # control.orientation.z = 1
        # control.name = "move_y"
        # control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # control.orientation_mode = InteractiveMarkerControl.FIXED
        # self.marker.controls.append(control)

        self.server.insert(self.marker, self.markerFeedback)
        self.menu_handler.apply(self.server, self.marker.name)

    def createInteractiveMarkerRGBD(self, x=0, y=0, z=0):
        self.marker = InteractiveMarker()
        self.marker.header.frame_id = self.parent
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        self.marker.scale = self.marker_scale

        self.marker.name = self.name
        self.marker.description = self.name + '_labeler'
        print('Creating IM with name ' + self.marker.name)

        # insert a box
        control = InteractiveMarkerControl()
        control.always_visible = True

        marker_box = Marker()
        marker_box.type = Marker.SPHERE
        marker_box.scale.x = self.marker.scale * 0.3
        marker_box.scale.y = self.marker.scale * 0.3
        marker_box.scale.z = self.marker.scale * 0.3
        marker_box.color.r = 1
        marker_box.color.g = 0
        marker_box.color.b = 0
        marker_box.color.a = 0.2

        control.markers.append(marker_box)
        self.marker.controls.append(control)

        self.marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        self.server.insert(self.marker, self.markerFeedback)
        self.menu_handler.apply(self.server, self.marker.name)

    # def createMarkerFrustum(self):
    #     marker = Marker()
    #
    #     marker.type = marker.LINE_LIST
    #     marker.action = marker.ADD
    #     marker.header.frame_id = frame_id
    #     # marker scale
    #     marker.scale.x = 0.01
    #
    #     # marker color
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #
    #     # marker orientaiton
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
    #
    #     # marker position
    #     marker.pose.position.x = 0.0
    #     marker.pose.position.y = 0.0
    #     marker.pose.position.z = 0.0
    #
    #     self.server.insert(self.marker_frustrum, self.markerFeedback)
    #     self.menu_handler.apply(self.server, self.marker_frustrum.name)
    #
    #
    # def calculateFrustrum(self, marker, w, h, f_x, f_y, frame_id):
    #     P1 = Point()
    #     P2 = Point()
    #     P3 = Point()
    #     P4 = Point()
    #     P5 = Point()
    #     P6 = Point()
    #     P7 = Point()
    #     P8 = Point()
    #
    #     Z_near = 0.3
    #     Z_far = 8
    #     fov_x = 2 * math.atan2(w, (2 * f_x))
    #     fov_y = 2 * math.atan2(h, (2 * f_y))
    #
    #     x_n = math.tan(fov_x / 2) * Z_near
    #     y_n = math.tan(fov_y / 2) * Z_near
    #
    #     x_f = math.tan(fov_x / 2) * Z_far
    #     y_f = math.tan(fov_y / 2) * Z_far
    #
    #     P1.x = -x_n
    #     P1.y = y_n
    #     P1.z = Z_near
    #
    #     P2.x = x_n
    #     P2.y = y_n
    #     P2.z = Z_near
    #
    #     P3.x = x_n
    #     P3.y = -y_n
    #     P3.z = Z_near
    #
    #     P4.x = -x_n
    #     P4.y = -y_n
    #     P4.z = Z_near
    #
    #     P5.x = -x_f
    #     P5.y = y_f
    #     P5.z = Z_far
    #
    #     P6.x = x_f
    #     P6.y = y_f
    #     P6.z = Z_far
    #
    #     P7.x = x_f
    #     P7.y = -y_f
    #     P7.z = Z_far
    #
    #     P8.x = -x_f
    #     P8.y = -y_f
    #     P8.z = Z_far
    #
    #     # marker line points
    #     marker.points = []
    #
    #     marker.points.append(P1)
    #     marker.points.append(P2)
    #
    #     marker.points.append(P2)
    #     marker.points.append(P3)
    #
    #     marker.points.append(P3)
    #     marker.points.append(P4)
    #
    #     marker.points.append(P4)
    #     marker.points.append(P1)
    #
    #     marker.points.append(P1)
    #     marker.points.append(P5)
    #     #
    #     marker.points.append(P2)
    #     marker.points.append(P6)
    #
    #     marker.points.append(P3)
    #     marker.points.append(P7)
    #
    #     marker.points.append(P4)
    #     marker.points.append(P8)
    #
    #     marker.points.append(P5)
    #     marker.points.append(P6)
    #
    #     marker.points.append(P6)
    #     marker.points.append(P7)
    #
    #     marker.points.append(P7)
    #     marker.points.append(P8)
    #
    #     marker.points.append(P8)
    #     marker.points.append(P5)
    #
    #
    #
    #     # return marker
