#!/usr/bin/env python3

"""
Handeye calibration from opencv. Eye in hand variant.
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

from functools import partial
import math
import time
import cv2
from matplotlib import cm
import numpy as np
import argparse
from atom_calibration.calibration.objective_function import getPointsInSensorAsNPArray
from atom_core.atom import getTransform
from atom_core.geometry import traslationRodriguesToTransform
from atom_core.optimization_utils import Optimizer
from copy import deepcopy
from colorama import init as colorama_init
import open3d as o3d
from scipy.spatial.transform import Rotation
import pprint

from atom_core.dataset_io import filterCollectionsFromDataset, loadResultsJSON
from atom_core.utilities import atomError, createLambdaExpressionsForArgs

pp = pprint.PrettyPrinter(indent=2)
colorama_init(autoreset=True)
np.set_printoptions(precision=3, suppress=True)


view = {
    "class_name": "ViewTrajectory", "interval": 29, "is_loop": False,
    "trajectory":
    [{"boundingbox_max": [1.4881374835968018, 1.0, 1.0],
      "boundingbox_min": [-0.059999999999999998, -0.83004301786422729, -0.40193906426429749],
      "field_of_view": 60.0,
      "front": [-0.95711769604701769, 0.0051956268381449571, 0.28965275999963758],
      "lookat": [0.72128191884594961, -0.50680060371821634, 0.18311132966112323],
      "up": [0.28882815786397104, -0.060368480020593036, 0.95547576727246641],
      "zoom": 0.61999999999999988}],
    "version_major": 1, "version_minor": 0}


# -------------------------------------------------------------------------------
#  FUNCTIONS
# -------------------------------------------------------------------------------
def pick_points(pcd, text):
    print(text + " [shift + left click]")
    print("Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()

    # TODO how to add a frame
    # frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    # vis.add_geometry(frame)

    vis.add_geometry(pcd)

    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


def main():

    # ---------------------------------------
    # Command line arguments
    # ---------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-json",
        "--json_file",
        help="Json file containing train input dataset.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-c", "--camera", help="Camera sensor name.", type=str, required=True
    )
    ap.add_argument("-l", "--lidar", help="Lidar sensor name.", type=str, required=True)
    ap.add_argument(
        "-p",
        "--pattern",
        help="Pattern to be used for calibration.",
        type=str,
        required=True,
    )
    ap.add_argument("-s", "--step", help="Step in lidar points.", type=int, default=1)

    # ap.add_argument("-hl", "--hand_link",
    # help="Name of coordinate frame of the hand.", type=str, required=True)
    # ap.add_argument("-bl", "--base_link",
    # help="Name of coordinate frame of the robot's base.", type=str, required=True)
    ap.add_argument(
        "-ctgt",
        "--compare_to_ground_truth",
        action="store_true",
        help="If the system being calibrated is simulated, directly compare the TFs to the ground truth.",
    )
    ap.add_argument(
        "-csf", "--collection_selection_function", default=None, type=str,
        help="A string to be evaluated into a lambda function that receives a collection name as input and "
        "returns True or False to indicate if the collection should be loaded (and used in the "
        "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
        "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.",)
    ap.add_argument(
        "-uic",
        "--use_incomplete_collections",
        action="store_true",
        default=False,
        help="Remove any collection which does not have a detection for all sensors.",
    )
    ap.add_argument(
        "-si",
        "--show_images",
        action="store_true",
        default=False,
        help="shows images for each camera",
    )
    ap.add_argument(
        "-mn",
        "--method_name",
        required=False,
        default="tsai",
        help="Hand eye method. One of ['tsai', 'park', 'horaud', 'andreff', 'daniilidis'].",
        type=str,
    )

    # these args have the selection functions as strings
    args_original = vars(ap.parse_args())
    # selection functions are now lambdas

    # print(args_original['collection_selection_function'])
    # exit(0)
    args = createLambdaExpressionsForArgs(args_original)

    # ---------------------------------------
    # Dataset loading and preprocessing
    # ---------------------------------------
    dataset, _ = loadResultsJSON(
        args["json_file"], args["collection_selection_function"]
    )

    # opencv can only process complete detections
    args["remove_partial_detections"] = True
    dataset = filterCollectionsFromDataset(dataset, args)

    dataset_ground_truth = deepcopy(dataset)  # make a copy before adding noise
    dataset_initial = deepcopy(dataset)  # store initial values

    colormap = cm.Set3(np.linspace(0, 1, len(dataset["collections"].keys())))

    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):

        # Get the 3D LiDAR labelled points for the given collection
        points_in_lidar = getPointsInSensorAsNPArray(collection_key, args["pattern"],
                                                     args["lidar"], "idxs", dataset)

        # Set the open3d point cloud
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points_in_lidar[0:3, :].transpose())

        # Annotate Top side

        # Pick points
        picked_pts_idxs = pick_points(
            pc, text="Please select at least two points from the top line.")
        print(picked_pts_idxs)

        picked_pts = points_in_lidar[0:3, picked_pts_idxs]
        print(picked_pts)

        # lidar_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        # entities.append(lidar_frame)

        # entities.append(pc)

        # o3d.visualization.draw_geometries(entities,
        #                                   zoom=view['trajectory'][0]['zoom'],
        #                                   front=view['trajectory'][0]['front'],
        #                                   lookat=view['trajectory'][0]['lookat'],
        #                                   up=view['trajectory'][0]['up'])


if __name__ == "__main__":
    main()
