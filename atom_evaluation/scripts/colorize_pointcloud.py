#!/usr/bin/env python

"""
Colorizes pointclouds with pcl-image pairs on a specific dataset
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import rospy
import ros_numpy
import struct
import json
import os
import cv2
import numpy as np
import argparse
import OptimizationUtils.utilities as opt_utilities
import atom_core.atom
from collections import OrderedDict
from atom_core.dataset_io import read_pcd, write_pcd
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

def walk(node):
    for key, item in node.items():
        if isinstance(item, dict):
            walk(item)
        else:
            if isinstance(item, np.ndarray) and key == 'data':  # to avoid saving images in the json
                del node[key]

            elif isinstance(item, np.ndarray):
                node[key] = item.tolist()
            pass


def rangeToImage(collection, ss, ts, tf, pts):
    # -- Convert limit points from velodyne to camera frame
    points_in_lidar = np.array(
        [[item[0] for item in pts], [item[1] for item in pts], [item[2] for item in pts],
         [1 for item in pts]], np.float)

    points_in_cam = np.dot(tf, points_in_lidar)

    # -- Project them to the image
    selected_collection_key = dataset['collections'].keys()[0]
    w, h = collection['data'][ts]['width'], dataset['collections'][selected_collection_key]['data'][ts]['height']
    K = np.ndarray((3, 3), buffer=np.array(dataset['sensors'][ts]['camera_info']['K']), dtype=np.float)
    D = np.ndarray((5, 1), buffer=np.array(dataset['sensors'][ts]['camera_info']['D']), dtype=np.float)

    pts_in_image, _, _ = opt_utilities.projectToCamera(K, D, w, h, points_in_cam[0:3, :])

    return pts_in_image


def rangeToColor(image, img_size, lidar_pts_in_image, lidar_points):
    points = []
    fields = []

    for i in range(0, lidar_pts_in_image.shape[1]):
        if int(lidar_pts_in_image[0, i]) > 0 and int(lidar_pts_in_image[0, i]) < img_size[1] and int(lidar_pts_in_image[1, i]) > 0 \
                and int(lidar_pts_in_image[1, i]) < img_size[0]:
            r, g, b = image[int(lidar_pts_in_image[1, i]), int(lidar_pts_in_image[0, i])]
        else:
            b, g, r = 255, 255, 255

        x, y, z = lidar_points[i]

        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]

    header = Header()
    header.frame_id = "lidar"
    pc2 = point_cloud2.create_cloud(header, fields, points)

    return pc2


# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node('colorize_pointclouds', anonymous=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str,
                    required=True)
    ap.add_argument("-ls", "--lidar_sensor", help="LiDAR sensor name.", type=str, required=True)
    ap.add_argument("-cs", "--camera_sensor", help="Camera sensor name.", type=str, required=True)

    # - Save args
    args = vars(ap.parse_args())
    lidar_sensor = args['lidar_sensor']
    camera_sensor = args['camera_sensor']

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads a json file containing the calibration
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    pc2s = []
    pc2_publishers = []
    from_frame = dataset['calibration_config']['sensors'][camera_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][lidar_sensor]['link']
    od = OrderedDict(sorted(dataset['collections'].items(), key=lambda t: int(t[0])))
    for collection_key, collection in od.items():
        # ---------------------------------------
        # --- Read point cloud from file
        # ---------------------------------------
        pcd_filename = os.path.dirname(json_file) + '/' + collection['data'][lidar_sensor]['data_file']
        frame_id = str(collection['data'][lidar_sensor]['header']['frame_id'])

        # Read point cloud
        msg = read_pcd(pcd_filename, cloud_header=None, get_tf=False)
        pc = ros_numpy.numpify(msg)
        points = np.zeros((pc.shape[0], 3))
        points[:, 0] = pc['x']
        points[:, 1] = pc['y']
        points[:, 2] = pc['z']

        # ---------------------------------------
        # --- Read image from file
        # ---------------------------------------
        cam_filename = os.path.dirname(json_file) + '/' + collection['data'][camera_sensor]['data_file']
        image = cv2.imread(cam_filename)
        img_size = image.shape

        # ---------------------------------------
        # --- Range to image projection
        # ---------------------------------------
        selected_collection_key = dataset['collections'].keys()[0]
        lidar2cam = atom_core.atom.getTransform(from_frame, to_frame,
                                                dataset['collections'][selected_collection_key]['transforms'])
        pts_in_image = rangeToImage(collection, lidar_sensor, camera_sensor, lidar2cam, points)

        # ---------------------------------------
        # --- Colorize and save point cloud
        # ---------------------------------------
        pc2 = rangeToColor(image, img_size, pts_in_image, points)
        pc2s.append(pc2)
        m_publisher = rospy.Publisher("/color_pcl_" + str(collection_key), PointCloud2, queue_size=1)
        pc2_publishers.append(m_publisher)

        break

    # Convert point clouds to numpy arrays
    print ('-----------')
    print ('Publishing colorized point clouds on "lidar" frame id... Open RVIZ for visualization.')
    print ('-----------')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for i in range(0, len(pc2_publishers)):
            pc2_publishers[i].publish(pc2s[i])

        rate.sleep()
