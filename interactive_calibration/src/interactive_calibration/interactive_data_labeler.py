#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import math
import threading
from __builtin__ import enumerate
from math import sqrt

import cv2
import rospy
from cv_bridge import CvBridge
from matplotlib import cm
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import *
from sensor_msgs.msg import *
import interactive_calibration.utilities
import numpy as np


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
        LaserScans: Semi-automated laelling. An rviz interactive marker is placed on the laser cluster which contains
                    the calibration pattern, and the pattern is tracked from there onward.
        PointCloud2: #TODO Tiago Madeira can you complete?
    """

    def __init__(self, server, menu_handler, sensor_dict, marker_scale, chess_numx, chess_numy):
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
        self.server = server
        self.menu_handler = menu_handler
        self.name = sensor_dict['_name']
        self.parent = sensor_dict['parent']
        self.topic = sensor_dict['topic']
        self.marker_scale = marker_scale
        self.received_first_msg = False
        self.labels = {'detected': False, 'idxs': []}
        self.lock = threading.Lock()
        self.numx = chess_numx
        self.numy = chess_numy

        # Get the type of message from the message topic of the sensor data, which is given as input. The message
        # type is used to define which labelling technique is used.
        self.msg_type_str, self.msg_type = interactive_calibration.utilities.getMessageTypeFromTopic(self.topic)
        print('msg_type_str is = ' + str(self.msg_type_str))

        # Subscribe to the message topic containing sensor data
        self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self.sensorDataReceivedCallback)

        # Handle the interactive labelling of data differently according to the sensor message types.
        if self.msg_type_str in ['LaserScan', 'PointCloud2']:
            # TODO parameters given from a command line input?
            self.threshold = 0.2  # pt to pt distance  to create new cluster (param  only for 2D LIDAR labelling)
            self.minimum_range_value = 0.3  # distance to assume range value valid (param only for 2D LIDAR labelling)
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.PointCloud2,
                                                             queue_size=0)  # publish a point cloud with the points
            # in the selected cluster
            self.publisher_clusters = rospy.Publisher(self.topic + '/clusters', sensor_msgs.msg.PointCloud2,
                                                      queue_size=0)  # publish a point cloud with coloured clusters
            self.createInteractiveMarker()  # interactive marker to label the calibration pattern cluster (one time)
            print('Created interactive marker.')
        elif self.msg_type_str in ['Image']:
            self.bridge = CvBridge()  # a CvBridge structure is needed to convert opencv images to ros messages.
            self.publisher_labelled_image = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.Image,
                                                            queue_size=0)  # publish
            # images with the detected chessboard overlaid onto the image.
        else:
            # We handle only know message types
            raise ValueError('Message type ' + self.msg_type_str + ' for topic ' + self.topic + 'is of an unknown '
                                                                                                'type.')
            # self.publisher = rospy.Publisher(self.topic + '/labeled', self.msg_type, queue_size=0)

    def sensorDataReceivedCallback(self, msg):
        self.lock.acquire()  # use semaphores to make sure the data is not being written on two sides simultaneously
        self.msg = msg  # make a local copy of sensor data
        self.labelData()  # label the data
        self.lock.release()  # release lock

    def labelData(self):
        # Detected and idxs values to False and [], to make sure we are not using information from a previous labelling
        self.labels['detected'] = False
        self.labels['idxs'] = []

        # Labelling process dependent of the sensor type
        if self.msg_type_str == 'LaserScan':  # 2D LIDARS -------------------------------------
            # For 2D LIDARS the process is the following: First cluster all the range data into clusters. Then,
            # associate one of the clusters with the calibration pattern by selecting the cluster which is closest to
            # the rviz interactive marker.

            clusters = []  # initialize cluster list to empty
            cluster_counter = 0  # init counter
            points = []  # init points

            # Compute cartesian coordinates
            xs, ys = interactive_calibration.utilities.laser_scan_msg_to_xy(self.msg)

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
                    distance = sqrt((xs[idx] - x) ** 2 + (ys[idx] - y) ** 2)
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
                    dist = sqrt((x_marker - x) ** 2 + (y_marker - y) ** 2)
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

        elif self.msg_type_str == 'Image':  # Cameras -------------------------------------------

            # Convert to opencv image and save image to disk
            image = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")

            # TODO cvtcolor only if image has 3 channels
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            self.found, corners = cv2.findChessboardCorners(image_gray, (self.numx, self.numy))
            cv2.drawChessboardCorners(image, (self.numx, self.numy), corners,
                                      self.found)  # Draw and display the corners

            if self.found is True:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(image_gray, corners, (self.numx, self.numy), (-1, -1), criteria)
                corners2_d = []
                for corner in corners2:
                    corners2_d.append({'x': float(corner[0][0]), 'y': float(corner[0][1])})

                x = int(round(corners2_d[0]['x']))
                y = int(round(corners2_d[0]['y']))
                cv2.line(image, (x, y), (x, y), (0, 255, 255), 20)

                # Update the dictionary with the labels
                self.labels['detected'] = True
                self.labels['idxs'] = corners2_d

            msg_out = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg_out.header.stamp = self.msg.header.stamp
            msg_out.header.frame_id = self.msg.header.frame_id
            self.publisher_labelled_image.publish(msg_out)

    def markerFeedback(self, feedback):
        # print(' sensor ' + self.name + ' received feedback')

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
