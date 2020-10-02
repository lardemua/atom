#!/usr/bin/env python

"""
ICP pcl-to-pcl alignment
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import atom_core.atom
import numpy as np
import json
import argparse
from sensor_msgs.msg import PointCloud2
from pcl import PointCloud


def load(path, format=None):
    """Load pointcloud from path.
    Currently supports PCD and PLY files.
    Format should be "pcd", "ply", or None to infer from the pathname.
    """
    format = _infer_format(path, format)
    p = PointCloud()
    try:
        loader = getattr(p, "_from_%s_file" % format)
    except AttributeError:
        raise ValueError("unknown file format %s" % format)
    if loader(_encode(path)):
        raise IOError("error while loading pointcloud from %r (format=%r)"
                      % (path, format))
    return p


def _encode(path):
    # Encode path for use in C++.
    if isinstance(path, bytes):
        return path
    else:
        return path.encode(sys.getfilesystemencoding())


def _infer_format(path, format):
    if format is not None:
        return format.lower()

    for candidate in ["pcd", "ply", "obj"]:
        if path.endswith("." + candidate):
            return candidate

    raise ValueError("Could not determine file format from pathname %s" % path)


def transformPCL(input_pcl, tf):
    np_pcl = np.asarray(input_pcl)

    transformed_pcl = []
    for idx in range(0, np_pcl.shape[0]):
        pt = np.zeros((4), np.float32)
        pt[0:3] = np.array(np_pcl[idx, :])
        pt[3] = 1

        tf_pt = np.dot(tf, pt.transpose())[0:3]
        transformed_pcl.append(tf_pt)

    return np.array(transformed_pcl)


def removeNan(input_np):
    output_np = []
    for idx in range(0, input_np.shape[0]):
        pt = input_np[idx]
        if not np.any(np.isnan(pt)):
            output_np.append(pt)

    return np.array(output_np)


if __name__ == "__main__":
    rospy.init_node('icp_aligner', anonymous=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing initial guess estimate.", type=str,
                    required=True)
    ap.add_argument("-sf", "--source_frame", help="Source frame link.", type=str, required=True)
    ap.add_argument("-tf", "--target_frame", help="Target frame link.", type=str, required=True)
    ap.add_argument("-rf", "--reference_frame", help="Reference frame link.", type=str, required=True)
    ap.add_argument("-spcl", "--source_pcl", help="Source point cloud from the two to be aligned", type=str,
                    required=True)
    ap.add_argument("-tpcl", "--target_pcl", help="Target point cloud from the two to be aligned", type=str,
                    required=True)

    # Save args
    args = vars(ap.parse_args())
    tpcl_path = args['target_pcl']
    spcl_path = args['source_pcl']
    target_frame = args['target_frame']
    source_frame = args['source_frame']
    reference_frame = args['reference_frame']

    # Load input json file
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Load point clouds from files
    tpcl = load(tpcl_path)
    spcl = load(spcl_path)

    # Get initial guess
    selected_collection_key = dataset['collections'].keys()[0]
    source2base = atom_core.atom.getTransform(source_frame, reference_frame,
                                              dataset['collections'][selected_collection_key]['transforms'])
    target2base = atom_core.atom.getTransform(target_frame, reference_frame,
                                              dataset['collections'][selected_collection_key]['transforms'])

    # Transform source and target point clouds to world using the initial guess
    tpcl_base = transformPCL(tpcl, np.linalg.inv(target2base))
    tpcl_base = PointCloud(tpcl_base.astype(np.float32))
    spcl_base = transformPCL(spcl, np.linalg.inv(source2base))
    spcl_base = removeNan(spcl_base)
    spcl_base = PointCloud(spcl_base.astype(np.float32))

    # # Apply icp alignment
    # print ('\n-----\nStarting ICP alignemnt ...\n-----\n')
    # icp = spcl_base.make_IterativeClosestPoint()
    # converged, transf, estimate, fitness = icp.icp(spcl_base, tpcl_base, max_iter=100)
    #
    # print('Has converged:' + str(converged) + ', score: ' + str(fitness))
    # print('Result:\n' + str(transf))
    #
    # print ('\n-----\nPublishing point clouds. You can visualize them on rviz.\n-----\n')
    # # Declare pcl publishers
    # source_pcl_pub = rospy.Publisher("/source_pcl", PointCloud2, queue_size=1)
    # target_pcl_pub = rospy.Publisher("/target_pcl", PointCloud2, queue_size=1)
    # aligned_pcl_pub = rospy.Publisher("/aligned_pcl", PointCloud2, queue_size=1)
    #
    # # Convert point clouds to numpy arrays
    # np_tpcl = np.asarray(tpcl_base)
    # np_spcl = np.asarray(spcl_base)
    # np_aligned = np.asarray(estimate)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     h = std_msgs.msg.Header()
    #     h.frame_id = 'velodyne'
    #     h.stamp = rospy.Time.now()
    #
    #     tpcl_cloud = pc2.create_cloud_xyz32(h, np_tpcl)
    #     spcl_cloud = pc2.create_cloud_xyz32(h, np_spcl)
    #     aligned_cloud = pc2.create_cloud_xyz32(h, np_aligned)
    #
    #     source_pcl_pub.publish(spcl_cloud)
    #     target_pcl_pub.publish(tpcl_cloud)
    #     aligned_pcl_pub.publish(aligned_cloud)
    #
    #     rate.sleep()
