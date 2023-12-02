#!/usr/bin/env python3

"""
Stereo calibration from opencv
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import numpy as np
import cv2
import argparse
import json
import os
import tf

from colorama import Style, Fore
from collections import OrderedDict
from atom_evaluation.utilities import atomicTfFromCalibration
from atom_core.atom import getTransform
from atom_core.dataset_io import saveAtomDataset
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosbag import Bag


def getTransformsFromTfStatic(dataset):
    bag = Bag(os.environ.get("ROS_BAGS") + dataset["calibration_config"]["bag_file"][len("$ROS_BAGS"):])
    topic = '/tf_static'
    dict = {}
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for tf in msg.transforms:
            tf_name = tf.header.frame_id+'-'+tf.child_frame_id
            dict[tf_name] = {}
            dict[tf_name]["child"] = tf.header.frame_id
            dict[tf_name]["parent"] = tf.child_frame_id
            dict[tf_name]["trans"] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            dict[tf_name]["quat"] = [tf.transform.rotation.x, tf.transform.rotation.y,
                                     tf.transform.rotation.z, tf.transform.rotation.w]
    return (dict)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing train input dataset.", type=str,
                    required=True)
    ap.add_argument("-si", "--show_images", help="If true the script shows images.", action='store_true', default=False)

    # Save args
    args = vars(ap.parse_args())
    json_file = args['json_file']
    show_images = args['show_images']

    # Read json file
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Get fabric transformation from bagfile
    new_tfs = getTransformsFromTfStatic(dataset)

    for collection_key, collection in dataset['collections'].items():
        for frame in new_tfs:
            dataset['collections'][collection_key]['transforms'][frame]['quat'] = new_tfs[frame]["quat"]
            dataset['collections'][collection_key]['transforms'][frame]['trans'] = new_tfs[frame]["trans"]

    # Save results to a json file
    filename_results_json = os.path.dirname(json_file) + '/factory_calibration.json'
    saveAtomDataset(filename_results_json, dataset)
