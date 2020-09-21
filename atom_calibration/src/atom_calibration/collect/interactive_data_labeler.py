#!/usr/bin/env python

# stdlib
import os

import atom_core.ros_utils
import math
import random
import threading

from __builtin__ import enumerate
from copy import deepcopy

# 3rd-party
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
from sensor_msgs.msg import *
from sensor_msgs.msg import PointField
from image_geometry import PinholeCameraModel

# local packages
from atom_calibration.collect import patterns
import atom_core.utilities

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

    def __init__(self, server, menu_handler, sensor_dict, marker_scale, calib_pattern, label_data = True):
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
        self.server = server
        self.menu_handler = menu_handler
        self.name = sensor_dict['_name']
        self.parent = sensor_dict['parent']
        self.topic = sensor_dict['topic']
        self.marker_scale = marker_scale
        self.received_first_msg = False
        self.labels = {'detected': False, 'idxs': []}
        self.lock = threading.Lock()

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

        # Handle the interactive labelling of data differently according to the sensor message types.
        if self.msg_type_str in ['LaserScan'] and self.label_data:
            # TODO parameters given from a command line input?
            self.threshold = 0.2  # pt to pt distance  to create new cluster (param  only for 2D LIDAR labelling)
            self.minimum_range_value = 0.3  # distance to assume range value valid (param only for 2D LIDAR labelling)
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled',
                                                             sensor_msgs.msg.PointCloud2,
                                                             queue_size=0)  # publish a point cloud with the points
            # in the selected cluster
            self.publisher_clusters = rospy.Publisher(self.topic + '/clusters', sensor_msgs.msg.PointCloud2,
                                                      queue_size=0)  # publish a point cloud with coloured clusters
            self.createInteractiveMarker()  # interactive marker to label the calibration pattern cluster (one time)
            print('Created interactive marker for laser scans.')
        elif self.msg_type_str in ['Image'] and self.label_data:
            self.bridge = CvBridge()  # a CvBridge structure is needed to convert opencv images to ros messages.
            self.publisher_labelled_image = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.Image,
                                                            queue_size=0)  # publish
            # images with the detected chessboard overlaid onto the image.

        elif self.msg_type_str in ['PointCloud2TIAGO'] and self.label_data:  # TODO, this will have to be revised later on Check #44
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.PointCloud2,
                                                             queue_size=0)  # publish a point cloud with the points
            self.createInteractiveMarkerRGBD()  # interactive marker to label the calibration pattern cluster (one time)
            self.bridge = CvBridge()
            self.publisher_labelled_depth_image = rospy.Publisher(self.topic + '/depth_image_labelled',
                                                                  sensor_msgs.msg.Image,
                                                                  queue_size=0)  # publish

            self.cam_model = PinholeCameraModel()
            topic_name = os.path.dirname(self.topic) + '/camera_info'
            rospy.loginfo('Waiting for for camera info message on topic' + str(topic_name))
            camera_info = rospy.wait_for_message('/top_center_rgbd_camera/depth/camera_info', CameraInfo)
            print('... received!')
            self.cam_model.fromCameraInfo(camera_info)
            print('Created interactive marker for point clouds.')

        elif self.msg_type_str in ['PointCloud2'] and self.label_data:  # Velodyne data (Andre Aguiar)
            self.publisher_selected_points = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.PointCloud2,
                                                             queue_size=0)  # publish a point cloud with the points
            self.createInteractiveMarkerRGBD(x=0.804, y=0.298,
                                             z=-0.109)  # interactive marker to label the calibration pattern
            # cluster (one time)

            # Labeler definitions
            # Hessian plane coefficients
            self.A = 0
            self.B = 0
            self.C = 0
            self.D = 0
            self.n_inliers = 0  # RANSAC number of inliers initialization
            self.number_iterations = 15  # RANSAC number of iterations
            self.ransac_threshold = 0.01  # RANSAC point-to-plane distance threshold to consider inliers
            # Chessboard point tracker distance threshold
            self.tracker_threshold = math.sqrt(((calib_pattern['dimension']['x'] - 1) * calib_pattern['size']) ** 2 + \
                                               ((calib_pattern['dimension']['y'] - 1) * calib_pattern['size']) ** 2)

            print('Created interactive marker for point clouds.')

        else:
            if self.label_data:
                # We handle only know message types
                raise ValueError(
                    'Message type ' + self.msg_type_str + ' for topic ' + self.topic + 'is of an unknown type.')
                # self.publisher = rospy.Publisher(self.topic + '/labeled', self.msg_type, queue_size=0)

        # Subscribe to the message topic containing sensor data
        print(self.msg_type)
        self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self.sensorDataReceivedCallback, queue_size=None)

    def sensorDataReceivedCallback(self, msg):
        self.lock.acquire()  # use semaphores to make sure the data is not being written on two sides simultaneously
        self.msg = msg  # make a local copy of sensor data
        # now = rospy.Time.now()
        if self.label_data:
            self.labelData  # label the data
        # rospy.loginfo('Labelling data for ' + self.name + ' took ' + str((rospy.Time.now() - now).to_sec()) + ' secs.')
        self.lock.release()  # release lock

    @property
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

        elif self.msg_type_str == 'Image':  # Cameras -------------------------------------------

            # Convert to opencv image and save image to disk
            image = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")

            result = self.pattern.detect(image, equalize_histogram=True)
            if result['detected']:
                c = []

                if result.has_key('ids'):
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

        elif self.msg_type_str == 'PointCloud2TIAGO':  # RGB-D pointcloud -------------------------------------------
            # TODO, this will have to be revised later on Check #44

            # print("Found point cloud!")

            tall = rospy.Time.now()

            # Get 3D coords
            t = rospy.Time.now()
            # points = pc2.read_points_list(self.msg, skip_nans=False, field_names=("x", "y", "z"))
            print('0. took ' + str((rospy.Time.now() - t).to_sec()))

            # Get the marker position
            x_marker, y_marker, z_marker = self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z  # interactive marker pose

            t = rospy.Time.now()
            # Project points
            print('x_marker=' + str(x_marker))
            print('y_marker=' + str(y_marker))
            print('z_marker=' + str(z_marker))
            seed_point = self.cam_model.project3dToPixel((x_marker, y_marker, z_marker))
            print('seed_point = ' + str(seed_point))
            if np.isnan(seed_point[0]):  # something went wrong, reposition marker on initial position and return
                self.marker.pose.position.x = 0
                self.marker.pose.position.y = 0
                self.marker.pose.position.z = 4
                self.menu_handler.reApply(self.server)
                self.server.applyChanges()
                rospy.logwarn('Could not project pixel, putting marker in home position.')
                return

            seed_point = (int(round(seed_point[0])), int(round(seed_point[1])))

            # Check if projection is inside the image
            x = seed_point[0]
            y = seed_point[1]
            if x < 0 or x >= self.cam_model.width or y < 0 or y >= self.cam_model.height:
                rospy.logwarn('Projection of point is outside of image. Not labelling point cloud.')
                return

            print('1. took ' + str((rospy.Time.now() - t).to_sec()))

            t = rospy.Time.now()
            # Wait for depth image message
            imgmsg = rospy.wait_for_message('/top_center_rgbd_camera/depth/image_rect', Image)

            print('2. took ' + str((rospy.Time.now() - t).to_sec()))

            t = rospy.Time.now()

            # img = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding="8UC1")
            img_raw = self.bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

            img = deepcopy(img_raw)
            img_float = img.astype(np.float32)
            img_float = img_float

            h, w = img.shape
            # print('img type = ' + str(img.dtype))
            # print('img_float type = ' + str(img_float.dtype))
            # print('img_float shape = ' + str(img_float.shape))

            mask = np.zeros((h + 2, w + 2, 1), np.uint8)

            # mask[seed_point[1] - 2:seed_point[1] + 2, seed_point[0] - 2:seed_point[0] + 2] = 255

            # PCA + Consensus + FloodFill ------------------

            # get 10 points around the seed
            # seed = {'x': seed_point[0], 'y': seed_point[1]}
            # pts = []
            # pts.append({'x': seed['x'], 'y': seed['y'] - 10})  # top neighbor
            # pts.append({'x': seed['x'], 'y': seed['y'] + 10})  # bottom neighbor
            # pts.append({'x': seed['x'] - 1, 'y': seed['y']})  # left neighbor
            # pts.append({'x': seed['x'] + 1, 'y': seed['y']})  # right neighbor
            #
            # def fitPlaneLTSQ(XYZ):
            #     (rows, cols) = XYZ.shape
            #     G = np.ones((rows, 3))
            #     G[:, 0] = XYZ[:, 0]  # X
            #     G[:, 1] = XYZ[:, 1]  # Y
            #     Z = XYZ[:, 2]
            #     (a, b, c), resid, rank, s = np.linalg.lstsq(G, Z)
            #     normal = (a, b, -1)
            #     nn = np.linalg.norm(normal)
            #     normal = normal / nn
            #     return (c, normal)
            #
            # data = np.random.randn(100, 3) / 3
            # data[:, 2] /= 10
            # c, normal = fitPlaneLTSQ(data)

            # out flood fill ------------------
            # to_visit = [{'x': seed_point[0], 'y': seed_point[1]}]
            # # filled = []
            # threshold = 0.05
            # filled_img = np.zeros((h, w), dtype=np.bool)
            # visited_img = np.zeros((h, w), dtype=np.bool)
            #
            # def isInsideBox(p, min_x, max_x, min_y, max_y):
            #     if min_x <= p['x'] < max_x and min_y <= p['y'] < max_y:
            #         return True
            #     else:
            #         return False
            #
            # def getNotVisitedNeighbors(p, min_x, max_x, min_y, max_y, img):
            #     neighbors = []
            #     tmp_neighbors = []
            #     tmp_neighbors.append({'x': p['x'], 'y': p['y'] - 1})  # top neighbor
            #     tmp_neighbors.append({'x': p['x'], 'y': p['y'] + 1})  # bottom neighbor
            #     tmp_neighbors.append({'x': p['x'] - 1, 'y': p['y']})  # left neighbor
            #     tmp_neighbors.append({'x': p['x'] + 1, 'y': p['y']})  # right neighbor
            #
            #     for idx, n in enumerate(tmp_neighbors):
            #         if isInsideBox(n, min_x, max_x, min_y, max_y) and not img[n['y'], n['x']] == True:
            #             neighbors.append(n)
            #
            #     return neighbors
            #
            # cv2.namedWindow('Filled', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('Visited', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('To Visit', cv2.WINDOW_NORMAL)
            # while to_visit != []:
            #     p = to_visit[0]
            #     # print('Visiting ' + str(p))
            #     range_p = img_float[p['y'], p['x']]
            #     to_visit.pop(0)  # remove p from to_visit
            #     # filled.append(p)  # append p to filled
            #     filled_img[p['y'], p['x']] = True
            #     # print(filled)
            #
            #     # compute neighbors of this point
            #     neighbors = getNotVisitedNeighbors(p, 0, w, 0, h, visited_img)
            #
            #     # print('neighbors ' + str(neighbors))
            #
            #     for n in neighbors:  # test if should propagate to neighbors
            #         range_n = img_float[n['y'], n['x']]
            #         visited_img[n['y'], n['x']] = True
            #
            #         if abs(range_n - range_p) <= threshold:
            #             # if not n in to_visit:
            #             to_visit.append(n)
            #
            #     # Create the mask image
            # to_visit_img = np.zeros((h, w), dtype=np.bool)
            # for p in to_visit:
            #     to_visit_img[p['y'], p['x']] = True
            #
            #
            # # print('To_visit ' + str(to_visit))
            #
            # cv2.imshow('Filled', filled_img.astype(np.uint8) * 255)
            # cv2.imshow('Visited', visited_img.astype(np.uint8) * 255)
            # cv2.imshow('To Visit', to_visit_img.astype(np.uint8) * 255)
            # key = cv2.waitKey(5)

            # --------------------------------

            img_float2 = deepcopy(img_float)
            cv2.floodFill(img_float2, mask, seed_point, 128, 80, 80,
                          8 | (128 << 8) | cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE)

            # Switch coords of seed point
            # mask[seed_point[1]-2:seed_point[1]+2, seed_point[0]-2:seed_point[0]+2] = 255

            tmpmask = mask[1:h + 1, 1:w + 1]

            cv2.namedWindow('tmpmask', cv2.WINDOW_NORMAL)
            cv2.imshow('tmpmask', tmpmask)

            def onMouse(event, x, y, flags, param):
                print("x = " + str(x) + ' y = ' + str(y) + ' value = ' + str(img_float2[y, x]))

            cv2.namedWindow('float', cv2.WINDOW_GUI_EXPANDED)
            cv2.setMouseCallback('float', onMouse, param=None)
            cv2.imshow('float', img_raw)
            key = cv2.waitKey(0)

            print('3. took ' + str((rospy.Time.now() - t).to_sec()))
            t = rospy.Time.now()

            # calculate moments of binary image
            M = cv2.moments(tmpmask)

            self.labels['detected'] = True
            print('4. took ' + str((rospy.Time.now() - t).to_sec()))
            t = rospy.Time.now()

            if M["m00"] != 0:
                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                red = deepcopy(img)
                # bmask =  tmpmask.astype(np.bool)

                print(tmpmask.shape)
                tmpmask = np.reshape(tmpmask, (480, 640))

                print(img.shape)

                red[tmpmask != 0] = red[tmpmask != 0] + 10000

                img = cv2.merge((img, img, red))

                img[cY - 2:cY + 2, cX - 2:cX + 2, 1] = 30000

                img[seed_point[1] - 2:seed_point[1] + 2, seed_point[0] - 2:seed_point[0] + 2, 0] = 30000

                # img[100:400, 20:150] = 255

                cv2.imshow("mask", img)
                cv2.waitKey(5)

                # msg_out = self.bridge.cv2_to_imgmsg(showcenter, encoding="passthrough")
                # msg_out.header.stamp = self.msg.header.stamp
                # msg_out.header.frame_id = self.msg.header.frame_id

                # self.publisher_labelled_depth_image.publish(msg_out)

                # coords = points[cY * 640 + cX]
                # print('coords' + str(coords))

                ray = self.cam_model.projectPixelTo3dRay((cX, cY))

                print('ray' + str(ray))
                print('img' + str(img_float.shape))

                print(type(cX))
                print(type(cY))
                print(type(ray))

                dist = float(img_float[cX, cY])
                print('dist = ' + str(dist))
                x = ray[0] * dist
                y = ray[1] * dist
                z = ray[2] * dist

                print('xyz = ' + str(x) + ' ' + str(y) + ' ' + str(z))

                # if not math.isnan(coords[0]):
                #     self.marker.pose.position.x = coords[0]
                #     self.marker.pose.position.y = coords[1]
                #     self.marker.pose.position.z = coords[2]
                #     self.menu_handler.reApply(self.server)
                #     self.server.applyChanges()

                if dist > 0.1:
                    # self.marker.pose.position.x = x
                    # self.marker.pose.position.y = y
                    # self.marker.pose.position.z = z
                    # self.menu_handler.reApply(self.server)
                    # self.server.applyChanges()
                    pass

            print('5. took ' + str((rospy.Time.now() - t).to_sec()))
            # idx = np.where(tmpmask == 100)
            # # Create tuple with (l, c)
            # pointcoords = list(zip(idx[0], idx[1]))
            #
            # points = pc2.read_points_list(self.msg, skip_nans=False, field_names=("x", "y", "z"))
            # tmppoints = []
            #
            # for coord in pointcoords:
            #     pointidx = (coord[0]) * 640 + (coord[1])
            #     tmppoints.append(points[pointidx])
            #
            # msg_out = createRosCloud(tmppoints, self.msg.header.stamp, self.msg.header.frame_id)
            #
            # self.publisher_selected_points.publish(msg_out)
            print('all. took ' + str((rospy.Time.now() - tall).to_sec()))

        elif self.msg_type_str == 'PointCloud2':  # 3D scan pointcloud (Andre Aguiar) ---------------------------------
            # Get the marker position (this comes from the shpere in rviz)
            x_marker, y_marker, z_marker = self.marker.pose.position.x, self.marker.pose.position.y, \
                                           self.marker.pose.position.z  # interactive marker pose

            # Extract 3D point from the LiDAR
            pc = ros_numpy.numpify(self.msg)
            points = np.zeros((pc.shape[0], 3))
            points[:, 0] = pc['x']
            points[:, 1] = pc['y']
            points[:, 2] = pc['z']

            # Extract the points close to the seed point from the entire PCL
            marker_point = np.array([[x_marker, y_marker, z_marker]])
            dist = scipy.spatial.distance.cdist(marker_point, points, metric='euclidean')
            pts = points[np.transpose(dist < self.tracker_threshold)[:, 0], :]
            idx = np.where(np.transpose(dist < self.tracker_threshold)[:, 0])[0]

            # Tracker - update seed point with the average of cluster to use in the next
            # iteration
            seed_point = []
            if 0 < len(pts):
                x_sum, y_sum, z_sum = 0, 0, 0
                for i in range(0, len(pts)):
                    x_sum += pts[i, 0]
                    y_sum += pts[i, 1]
                    z_sum += pts[i, 2]
                seed_point.append(x_sum / len(pts))
                seed_point.append(y_sum / len(pts))
                seed_point.append(z_sum / len(pts))

            # RANSAC - eliminate the tracker outliers
            number_points = pts.shape[0]
            if number_points == 0:
                return []
            # RANSAC iterations
            for i in range(0, self.number_iterations):
                # Randomly select three points that cannot be coincident
                # nor collinear
                while True:
                    idx1 = random.randint(0, number_points - 1)
                    idx2 = random.randint(0, number_points - 1)
                    idx3 = random.randint(0, number_points - 1)
                    pt1, pt2, pt3 = pts[[idx1, idx2, idx3], :]
                    # Compute the norm of position vectors
                    ab = np.linalg.norm(pt2 - pt1)
                    bc = np.linalg.norm(pt3 - pt2)
                    ac = np.linalg.norm(pt3 - pt1)
                    # Check if points are colinear
                    if (ab + bc) == ac:
                        continue
                    # Check if are coincident
                    if idx2 == idx1:
                        continue
                    if idx3 == idx1 or idx3 == idx2:
                        continue

                    # If all the conditions are satisfied, we can end the loop
                    break

                # ABC Hessian coefficients and given by the external product between two vectors lying on hte plane
                A, B, C = np.cross(pt2 - pt1, pt3 - pt1)
                # Hessian parameter D is computed using one point that lies on the plane
                D = - (A * pt1[0] + B * pt1[1] + C * pt1[2])
                # Compute the distance from all points to the plane
                # from https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
                distances = abs((A * pts[:, 0] + B * pts[:, 1] + C * pts[:, 2] + D)) / (
                    math.sqrt(A * A + B * B + C * C))
                # Compute number of inliers for this plane hypothesis.
                # Inliers are points which have distance to the plane less than a tracker_threshold
                num_inliers = (distances < self.ransac_threshold).sum()
                # Store this as the best hypothesis if the number of inliers is larger than the previous max
                if num_inliers > self.n_inliers:
                    self.n_inliers = num_inliers
                    self.A = A
                    self.B = B
                    self.C = C
                    self.D = D

            # Extract the inliers
            distances = abs((self.A * pts[:, 0] + self.B * pts[:, 1] + self.C * pts[:, 2] + self.D)) / \
                        (math.sqrt(self.A * self.A + self.B * self.B + self.C * self.C))
            inliers = pts[np.where(distances < self.ransac_threshold)]
            # Create dictionary [pcl point index, distance to plane] to select the pcl indexes of the inliers
            idx_map = dict(zip(idx, distances))
            final_idx = []
            for key in idx_map:
                if idx_map[key] < self.ransac_threshold:
                    final_idx.append(key)
            # -------------------------------------- End of RANSAC ----------------------------------------- #

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

            # Reset the number of inliers to have a fresh start on the next interation
            self.n_inliers = 0

            # Update the dictionary with the labels (to be saved if the user selects the option)
            self.labels['detected'] = True
            self.labels['idxs'] = final_idx

            # Update the interactive marker pose
            self.marker.pose.position.x = seed_point[0]
            self.marker.pose.position.y = seed_point[1]
            self.marker.pose.position.z = seed_point[2]
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

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
