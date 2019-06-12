#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import math
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

# return Fore.GREEN + self.parent + Style.RESET_ALL + ' to ' + Fore.GREEN + self.child + Style.RESET_ALL + ' (' + self.joint_type + ')'

class LaserScanCluster:

    def __init__(self, cluster_count, idx):
        self.cluster_count = cluster_count
        self.idxs = [idx]

    def pushIdx(self, idx):
        self.idxs.append(idx)


class InteractiveDataLabeler:

    def __init__(self, server, menu_handler, sensor_dict):
        print('Creating an InteractiveDataLabeler for sensor ' + str(sensor_dict['_name']))

        self.server = server
        self.menu_handler = menu_handler
        self.name = sensor_dict['_name']
        self.parent = sensor_dict['parent']
        self.topic = sensor_dict['topic']
        self.received_first_msg = False
        self.labels = {'detected': False, 'idxs': []}

        self.createInteractiveMarker()  # create interactive marker
        print('Created interactive marker.')

        self.msg_type_str, self.msg_type = interactive_calibration.utilities.getMessageTypeFromTopic(self.topic)
        self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self.sensorDataReceivedCallback)

        print('msg_type_str is = ' + str(self.msg_type_str))
        if self.msg_type_str in ['LaserScan', 'PointCloud2']:
            self.publisher = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.PointCloud2, queue_size=0)
        elif self.msg_type_str in ['Image']:
            self.bridge = CvBridge()
            self.publisher = rospy.Publisher(self.topic + '/labeled', sensor_msgs.msg.Image, queue_size=0)
        else:
            self.publisher = rospy.Publisher(self.topic + '/labeled', self.msg_type, queue_size=0)

        rospy.Timer(rospy.Duration(.1), self.timerCallback)

    def timerCallback(self, event):

        if not self.received_first_msg:  # nothing to do if no msg received
            return None

        # if no chessboard is detected, the labels are empty
        self.labels['detected'] = False
        self.labels['idxs'] = []

        if self.msg_type_str == 'LaserScan':

            # Create clusters in 2D scan
            clusters = []

            ranges = self.msg.ranges
            threshold = .2  # half a meter
            cluster_counter = 0
            points = []
            xs, ys = interactive_calibration.utilities.laser_scan_msg_to_xy(self.msg)

            for idx, r in enumerate(ranges):

                if idx == 0:
                    clusters.append(LaserScanCluster(cluster_counter, idx))
                else:
                    if abs(ranges[idx - 1] - r) > threshold:  # new cluster
                        cluster_counter += 1
                        clusters.append(LaserScanCluster(cluster_counter, idx))
                    else:
                        clusters[-1].pushIdx(idx)

            # print('Laser scan split into ' + str(len(clusters)) + ' clusters')

            # Find out which cluster is closer to the marker
            x_marker = self.marker.pose.position.x
            y_marker = self.marker.pose.position.y
            z_marker = self.marker.pose.position.z
            distance_to_point = 9999999
            idx_closest_point = None
            x_closest_point = 0
            y_closest_point = 0

            z = 0
            for idx, (x, y) in enumerate(zip(xs, ys)):
                dist = sqrt((x_marker - x) ** 2 + (y_marker - y) ** 2 + (z_marker - z) ** 2)

                if dist < distance_to_point:
                    distance_to_point = dist
                    idx_closest_point = idx
                    x_closest_point = x
                    y_closest_point = y

            idx_closest_cluster = 0
            for idx, cluster in enumerate(clusters):
                if idx_closest_point in cluster.idxs:
                    idx_closest_cluster = idx

            num_clusters = len(clusters)
            # print('Closest cluster is ' + str(idx_closest_cluster))

            # Find the coordinate of the middle point in the closest cluster and bring the marker to that point
            # r = ranges[idx_closest_point]
            # theta = self.msg.angle_min + idx_closest_point * self.msg.angle_increment
            closest_cluster = clusters[idx_closest_cluster]
            idx_middle_point = int((closest_cluster.idxs[0] + closest_cluster.idxs[-1]) / 2)
            r = ranges[idx_middle_point]
            theta = self.msg.angle_min + idx_middle_point * self.msg.angle_increment
            self.marker.pose.position.x = r * math.cos(theta)
            self.marker.pose.position.y = r * math.sin(theta)
            self.marker.pose.position.z = 0
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

            # Update the dictionary with the labels
            self.labels['detected'] = True
            self.labels['idxs'] = clusters[idx_closest_cluster].idxs

            # Create point cloud message with the colored clusters (just for debugging)
            # cmap = cm.Pastel2(np.linspace(0, 1, num_clusters))
            # cmap = cm.Accent(np.linspace(0, 1, num_clusters))
            # points = []
            # for cluster in clusters:
            #     for idx in cluster.idxs:
            #         x = xs[idx]
            #         y = ys[idx]
            #         z = 0
            #         r = int(cmap[cluster.cluster_count, 0] * 255.0)
            #         g = int(cmap[cluster.cluster_count, 1] * 255.0)
            #         b = int(cmap[cluster.cluster_count, 2] * 255.0)
            #         a = 255
            #         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            #         pt = [x, y, z, rgb]
            #         points.append(pt)

            # Create point cloud message with the colored clusters (just for debugging)
            points = []
            for idx in clusters[idx_closest_cluster].idxs:
                x_marker = xs[idx]
                y_marker = ys[idx]
                z_marker = 0
                r = int(0 * 255.0)
                g = int(0 * 255.0)
                b = int(1 * 255.0)
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x_marker, y_marker, z_marker, rgb]
                points.append(pt)

            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
            header = Header()
            header.frame_id = self.parent
            header.stamp = self.msg.header.stamp
            pc_msg = point_cloud2.create_cloud(header, fields, points)
            self.publisher.publish(pc_msg)

        elif self.msg_type_str == 'Image':

            # Convert to opencv image and save image to disk
            image = self.bridge.imgmsg_to_cv2(self.msg, "bgr8")

            # TODO cvtcolor only if image has 3 channels
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            self.found, corners = cv2.findChessboardCorners(image_gray, (8, 6))
            cv2.drawChessboardCorners(image, (8, 6), corners, self.found)  # Draw and display the corners

            if self.found is True:
                # print('Found chessboard for ' + self.name)
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(image_gray, corners, (8, 6), (-1, -1), criteria)
                corners2_d = []
                print(corners2_d)
                for corner in corners2:
                    corners2_d.append({'x': float(corner[0][0]), 'y': float(corner[0][1])})

                x = int(round(corners2_d[0]['x']))
                y = int(round(corners2_d[0]['y']))
                cv2.line(image, (x, y), (x, y), (0, 255, 255), 20)

                # if self.name == 'top_right_camera':
                #     print('corners2_d =\n' + str(corners2_d))

                # cv2.imshow(self.name, image)
                # cv2.imwrite('/tmp/img.png', image)
                # cv2.waitKey(0)

                # Update the dictionary with the labels
                self.labels['detected'] = True
                self.labels['idxs'] = corners2_d

            msg_out = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            msg_out.header.stamp = self.msg.header.stamp
            msg_out.header.frame_id = self.msg.header.frame_id
            self.publisher.publish(msg_out)


    def sensorDataReceivedCallback(self, msg):
        self.msg = msg
        self.received_first_msg = True

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
        self.marker.scale = .5

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
