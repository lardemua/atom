#!/usr/bin/env python3

"""
Handeye calibration from opencv. Eye in hand variant.
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

from functools import partial
import json
import math
import os
import time
import cv2
from matplotlib import cm
from networkx import draw_circular
import numpy as np
import argparse

from atom_evaluation.utilities import averageTransforms
from atom_calibration.calibration.objective_function import getPointsInSensorAsNPArray
from atom_core.atom import getTransform
from atom_core.geometry import traslationRodriguesToTransform
from atom_core.optimization_utils import Optimizer
from copy import deepcopy
from colorama import Fore, Style, init as colorama_init
import open3d as o3d
from scipy.spatial.transform import Rotation
import pprint

from atom_core.dataset_io import addNoiseToInitialGuess, addNoiseToTF, filterCollectionsFromDataset, loadResultsJSON
from atom_core.utilities import atomError, createLambdaExpressionsForArgs

pp = pprint.PrettyPrinter(indent=2)
colorama_init(autoreset=True)
np.set_printoptions(precision=3, suppress=True)


view = {
    "class_name": "ViewTrajectory", "interval": 29, "is_loop": False,
    "trajectory":
    [{"boundingbox_max": [1.0, 1.0, 1.0],
      "boundingbox_min":
      [-0.059999999999999998, -0.059999999999999998, -0.059999999999999998],
      "field_of_view": 60.0,
      "front": [0.10333878362207262, -0.3603556287500056, -0.92707330704087199],
      "lookat": [0.61239061772808823, 0.050664475984916711, 0.47587434236622195],
      "up": [-0.31148783770537486, -0.89690418403010197, 0.31390796681659294],
      "zoom": 1.0400000000000003}],
    "version_major": 1, "version_minor": 0}

# -------------------------------------------------------------------------------
#  FUNCTIONS
# -------------------------------------------------------------------------------


def ku(a, b):
    true_points = a
    mapping_points = b

    mapped_centroid = np.average(mapping_points, axis=0)
    true_centroid = np.average(true_points, axis=0)

    h = mapping_points.T @ true_points
    u, s, vt = np.linalg.svd(h)
    v = vt.T

    d = np.linalg.det(v @ u.T)
    e = np.array([[1, 0, 0], [0, 1, 0], [0, 0, d]])

    r = v @ e @ u.T

    tt = true_centroid - np.matmul(r, mapped_centroid)

    transform = np.eye(4)
    transform[0:3, 0:3] = r
    transform[0:3, 3] = tt

    return transform


def kabsch_umeyama(A, B):
    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = ((A - EA).T @ (B - EB)) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U) * np.linalg.det(VT))
    S = np.diag([1] * (m - 1) + [d])

    R = U @ S @ VT
    c = VarA / np.trace(np.diag(D) @ S)
    t = EA - c * R @ EB

    print(R)
    print(t)

    transform = np.eye(4)
    transform[0:3, 0:3] = R

    print(R)
    print(t)
    transform[0:3, 3] = t

    return transform, c

# Implements Kabsch algorithm - best fit.
# Supports scaling (umeyama)
# Compares well to SA results for the same data.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns s,R,t
# s = scale B to A
# R = 3x3 rotation matrix (B to A)
# t = 3x1 translation vector (B to A)


def rigid_transform_3D(A, B, scale=False):
    # From https://gist.github.com/oshea00/dfb7d657feca009bf4d095d4cb8ea4be

    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    if scale:
        H = np.transpose(BB) * AA / N
    else:
        # H = np.transpose(BB) * AA
        # NOTE in
        # they say the line above should be
        H = np.transpose(AA) @ BB

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = Vt.T * U.T

    if scale:
        varA = np.var(A, axis=0).sum()
        c = 1 / (1 / varA * np.sum(S))  # scale factor
        t = -R * (centroid_B.T * c) + centroid_A.T
    else:
        c = 1
        print(-R @ centroid_B.T)
        t = -R @ centroid_B.T + centroid_A.T

    transform = np.eye(4)
    transform[0:3, 0:3] = R

    print(R)
    print(t)
    transform[0:3, 3] = t
    return c, transform


def get4x4TransformFromXYZWPR(xyzwpr):

    T = np.zeros((4, 4), float)

    # Set rotation
    r = Rotation.from_euler("xyz", [xyzwpr["w"], xyzwpr["p"], xyzwpr["r"]], degrees=False)
    T[0:3, 0:3] = r.as_matrix()

    # Set translation
    T[0:3, 3] = [xyzwpr["x"], xyzwpr["y"], xyzwpr["z"]]

    T[3, 3] = 1.0  # Set the 1 on the bottom right corner

    return T


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm


def draw_cylinder(entities, x, y, z, radius=0.03, height=0.05, color=(0, 0, 1)):
    sphere_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    sphere_cylinder = sphere_cylinder.translate((x, y, z))
    sphere_cylinder.paint_uniform_color(color)
    entities.append(sphere_cylinder)


def draw_sphere(entities, x, y, z, radius=0.03, color=(0, 0, 1)):
    sphere_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere_mesh = sphere_mesh.translate((x, y, z))
    sphere_mesh.paint_uniform_color(color)
    entities.append(sphere_mesh)


def fitPlaneLTSQ(XYZ):
    (rows, cols) = XYZ.shape
    G = np.ones((rows, 3))
    G[:, 0] = XYZ[:, 0]  # X
    G[:, 1] = XYZ[:, 1]  # Y
    Z = XYZ[:, 2]
    (a, b, c), resid, rank, s = np.linalg.lstsq(G, Z, rcond=None)
    normal = (a, b, -1)
    nn = np.linalg.norm(normal)
    normal = normal / nn
    return (c, normal)


def main():

    # ---------------------------------------
    # Command line arguments
    # ---------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-d",         "--dataset_file", type=str, required=True,
                    help="Json file containing train input dataset.",
                    )
    ap.add_argument("-c", "--camera", type=str, required=True, help="Camera sensor name.")
    ap.add_argument("-l", "--lidar", help="Lidar sensor name.", type=str, required=True)
    ap.add_argument("-p",        "--pattern",         type=str,         required=True,
                    help="Pattern to be used for calibration.",)
    ap.add_argument("-s", "--step", help="Step in lidar points.", type=int, default=1)
    ap.add_argument(
        "-ctgt", "--compare_to_ground_truth", action="store_true",
        help="If the system being calibrated is simulated, directly compare the TFs to the ground truth.",)
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
        help="Remove any collection which does not have a detection for all sensors.",)
    ap.add_argument("-si", "--show_images", action="store_true", default=False,
                    help="shows images for each camera",)
    ap.add_argument("-mn", "--method_name", required=False, default='icp_based', type=str,
                    help="Method to use in Dahll corner matching. One of ['icp_based', 'closed_form'].",)
    ap.add_argument(
        "-nig", "--noisy_initial_guess", nargs=2, metavar=("translation", "rotation"),
        help="Magnitude of noise to add to the initial guess atomic transformations set before starting optimization [meters, radians].",
        type=float, default=[0.0, 0.0],)
    ap.add_argument("-ss", "--sample_seed", help="Sampling seed", type=int)

    # these args have the selection functions as strings
    args_original = vars(ap.parse_args())
    # selection functions are now lambdas

    args = createLambdaExpressionsForArgs(args_original)

    # ---------------------------------------
    # Dataset loading and preprocessing
    # ---------------------------------------
    dataset, _ = loadResultsJSON(args["dataset_file"], args["collection_selection_function"])

    # opencv can only process complete detections
    args["remove_partial_detections"] = True
    dataset = filterCollectionsFromDataset(dataset, args)

    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # We only need to get one collection because optimized transformations are static, which means they are the same for all collections. Let's select the first key in the dictionary and always get that transformation.
    selected_collection_key = list(dataset["collections"].keys())[0]
    print("Selected collection key is " + str(selected_collection_key))

    # ---------------------------------------
    # --- Add noise to the transformations  to be calibrated.
    # ---------------------------------------
    dataset_ground_truth = deepcopy(dataset)  # make a copy before adding noise

    addNoiseToInitialGuess(dataset, args, selected_collection_key)
    dataset_initial = deepcopy(dataset)  # store initial values

    # ---------------------------------------
    # Verifications
    # ---------------------------------------
    available_methods = ['icp_based', 'closed_form']
    if args['method_name'] not in available_methods:
        atomError('Unknown method. Select one from ' + str(available_methods))

    # ------------------------------------------
    # Set the ground truth initial pose
    # ------------------------------------------
    camera_T_lidar_gt = getTransform(
        from_frame=dataset["calibration_config"]["sensors"][args["camera"]]["link"],
        to_frame=dataset["calibration_config"]["sensors"][args["lidar"]]["link"],
        transforms=dataset_ground_truth["collections"][selected_collection_key]["transforms"],)

    camera_T_lidar_initial = getTransform(
        from_frame=dataset["calibration_config"]["sensors"][args["camera"]]["link"],
        to_frame=dataset["calibration_config"]["sensors"][args["lidar"]]["link"],
        transforms=dataset_initial["collections"][selected_collection_key]["transforms"],)

    # ---------------------------------------
    # Pattern configuration
    # ---------------------------------------
    nx = dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                               ]['dimension']['x']
    ny = dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                               ]['dimension']['y']
    square = dataset['calibration_config']['calibration_patterns'][args['pattern']]['size']
    pts_3d = np.zeros((nx * ny, 3), np.float32)
    # set of coordinates (w.r.t. the pattern frame) of the corners
    pts_3d[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    number_of_corners = int(nx) * int(ny)

    # ---------------------------------------
    # --- Get intrinsic data for the sensor
    # ---------------------------------------
    # Source sensor
    K = np.zeros((3, 3), np.float32)
    D = np.zeros((5, 1), np.float32)
    K[0, :] = dataset['sensors'][args['camera']]['camera_info']['K'][0:3]
    K[1, :] = dataset['sensors'][args['camera']]['camera_info']['K'][3:6]
    K[2, :] = dataset['sensors'][args['camera']]['camera_info']['K'][6:9]
    D[:, 0] = dataset['sensors'][args['camera']]['camera_info']['D'][0:5]

    # ----------------------------------------------------
    # Load lidar annotations
    # ----------------------------------------------------
    annotation_file = os.path.dirname(args['dataset_file']) + "/annotation_" + \
        args['lidar'] + ".json"
    if os.path.exists(annotation_file):
        print('Found anotation file at ' + annotation_file + ' ... loading.')
        f = open(annotation_file, 'r')
        annotations = json.load(f)

        if not annotations['_metadata']['version'] == '2.0':
            print(Fore.RED +
                  'This annotations file does not have the correct version. Exiting.' +
                  Style.RESET_ALL)
    else:
        atomError(
            'Could not find lidar annotation file named ' + annotation_file +
            '\nAre you ran the annotate_pattern_borders_in_lidar script?\nSee https://github.com/lardemua/atom/issues/958 .')

    # ------------------------------------------
    # Compute the physical corners from the camera in 3D
    # ------------------------------------------
    camera_T_pattern_lst = []
    corners_in_camera_lst = []
    for collection_key, collection in dataset['collections'].items():

        # Pattern not detected by sensor in collection
        if not collection['labels'][args['pattern']][args['camera']]['detected']:
            continue

        # ---------------------------------------
        # Get c_T_p using pattern detection
        # ---------------------------------------
        # we will us the solve pnp function from opencv.
        # This requires a np array of the pattern corners 3d coordinates
        # and another np array of the correponding 2d pixels.
        # 3d coordinates were extracted previously, so lets go to 2d..
        pts_2d = np.ones((number_of_corners, 2), np.float32)
        for idx, point in enumerate(collection['labels'][args['pattern']][args['camera']]['idxs']):
            pts_2d[idx, 0] = point['x']
            pts_2d[idx, 1] = point['y']

        retval, rvec, tvec = cv2.solvePnP(pts_3d, pts_2d, K, D)
        if not retval:
            raise atomError('solvePnP failed.')

        # Convert to 4x4 transform
        camera_T_pattern = traslationRodriguesToTransform(tvec, rvec)
        camera_T_pattern_lst.append(camera_T_pattern)

        pts_in_camera = np.zeros((4, 4), dtype=float)

        pattern = dataset['calibration_config']['calibration_patterns'][args['pattern']]

        # Top right corner
        x = nx * pattern['size'] + pattern['border_size']['x']
        y = 0 - pattern['size'] - pattern['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 0] = pt_in_camera

        # Bottom right corner
        x = nx * pattern['size'] + pattern['border_size']['x']
        y = ny * pattern['size'] + pattern['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 1] = pt_in_camera

        # Bottom left corner
        x = 0 - pattern['size'] - pattern['border_size']['x']
        y = ny * pattern['size'] + pattern['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 2] = pt_in_camera

        # Top left corner
        x = 0 - pattern['size'] - pattern['border_size']['x']
        y = 0 - pattern['size'] - pattern['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 3] = pt_in_camera

        # append to corners
        corners_in_camera_lst.append(pts_in_camera)

    # ------------------------------------------
    # Run calibration
    # ------------------------------------------

    # Estimate one transformation per collection
    camera_T_lidar_lst = []
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):

        # Camera corners to required format
        corners_in_camera = corners_in_camera_lst[idx]
        # print('corners_in_lidar=\n' + str(corners_in_lidar))

        # Lidar corners to required format
        corners_in_lidar = np.zeros((4, 4))
        for idx, (intersection_key, intersection) in enumerate(
                annotations[collection_key]['intersections'].items()):

            corners_in_lidar[0, idx] = intersection['x']
            corners_in_lidar[1, idx] = intersection['y']
            corners_in_lidar[2, idx] = intersection['z']
            corners_in_lidar[3, idx] = 1.0
        # print('corners_in_camera=\n' + str(corners_in_camera))

        # compute averages
        average_corners_in_camera = np.reshape(np.mean(corners_in_camera, axis=1), (4, 1))
        average_corners_in_lidar = np.reshape(np.mean(corners_in_lidar, axis=1), (4, 1))

        # output should be estimated camera_T_lidar tranform
        if args['method_name'] == 'icp_based':

            # prepare two open3d point clouds and then use open3d's icp
            camera_corners = o3d.geometry.PointCloud()
            camera_corners.points = o3d.utility.Vector3dVector(
                corners_in_camera[0: 3, :].transpose())

            lidar_corners = o3d.geometry.PointCloud()
            lidar_corners.points = o3d.utility.Vector3dVector(corners_in_lidar.transpose())

            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-06, relative_rmse=1e-06, max_iteration=300)

            reg_p2p = o3d.pipelines.registration.registration_icp(
                lidar_corners, camera_corners, 100.0, camera_T_lidar_initial,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(), criteria=criteria)

            print('Collection ' + collection_key)
            print(reg_p2p)

            camera_T_lidar_lst.append(reg_p2p.transformation)
            print(
                'Collection ' + collection_key + ' estimated transform=\n' +
                str(reg_p2p.transformation))

            # Estimate final camera_T_lidar transformation using average as proposed in Dahll2017
            print('gt transform=\n' + str(camera_T_lidar_gt))
            camera_T_lidar = averageTransforms(camera_T_lidar_lst)

        elif args['method_name'] == 'closed_form':

            # align to zero
            zero_aligned_corners_in_camera = corners_in_camera - \
                np.repeat(average_corners_in_camera, 4, axis=1)

            zero_aligned_corners_in_lidar = corners_in_lidar - \
                np.repeat(average_corners_in_lidar, 4, axis=1)

            # Use Kabsh algorithm to estimate rotation between corners
            # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.align_vectors.html
            # we need N,3 lists for the input vectors a and b
            a = np.array((4, 3))
            a = corners_in_camera[0:3, :].T
            # a = zero_aligned_corners_in_camera[0:3, :].T
            b = np.array((4, 3))
            b = corners_in_lidar[0:3, :].T
            # b = zero_aligned_corners_in_lidar[0:3, :].T

            rot, rssd, sens = Rotation.align_vectors(a, b, return_sensitivity=True)

            rotation = rot.as_matrix()

            # transform2, c = kabsch_umeyama(a, b)
            transform2 = ku(a, b)

            # c, transform2 = rigid_transform_3D(a, b, scale=False)
            print('transform2 =\n' + str(transform2))

            # Compute the translation using eq 8 from the paper
            # https://arxiv.org/pdf/1705.09785

            transform_without_translation = np.eye(4)
            transform_without_translation[0:3, 0:3] = rotation
            translation = average_corners_in_camera - transform_without_translation @ average_corners_in_lidar

            # Build 4x4 tranformation
            transform = np.eye(4)
            transform[0:3, 0:3] = rotation
            transform[0:3, 3] = translation[0:3, 0]

            camera_T_lidar_lst.append(transform2)

            print('Collection ' + collection_key + ' estimated transform=\n' + str(transform))

    # Estimate final camera_T_lidar transformation using average as proposed in Dahll2017
    print('gt transform=\n' + str(camera_T_lidar_gt))
    camera_T_lidar = averageTransforms(camera_T_lidar_lst)

    # ------------------------------------------
    # Define visualization
    # ------------------------------------------
    entities = []
    colormap = cm.Set3(np.linspace(0, 1, len(dataset["collections"].keys())))

    camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    entities.append(camera_frame)

    # # Add pattern frames
    # for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
    #     pattern_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    #     pattern_frame = pattern_frame.transform(camera_T_pattern_lst[idx])
    #     entities.append(pattern_frame)

    # Draw lidar points (using estimated transform)
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
        # Get the 3D LiDAR labelled points for the given collection
        points_in_lidar = getPointsInSensorAsNPArray(collection_key, args["pattern"],
                                                     args["lidar"], "idxs", dataset)

        # Transform points from lidar to camera coordinate frame
        points_in_camera = camera_T_lidar @ points_in_lidar

        # After calibration
        pc_in_camera = o3d.geometry.PointCloud()
        pc_in_camera.points = o3d.utility.Vector3dVector(points_in_camera[0:3, :].transpose())

        color = colormap[idx, 0:3]
        pc_in_camera.paint_uniform_color(color)

        entities.append(pc_in_camera)

    if args['method_name'] == 'closed_form':
        # Draw average camera corners
        for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
            color = colormap[idx, 0:3]
            x, y, z = average_corners_in_camera[0:3, 0]
            draw_sphere(entities, x, y, z, radius=0.06, color=color)

        # Draw average lidar corners
        for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
            color = colormap[idx, 0:3]

            pts = camera_T_lidar @ average_corners_in_lidar
            x, y, z = pts[0:3, 0]
            draw_cylinder(entities, x, y, z, radius=0.06, height=0.2, color=color)

    # Draw camera corners
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):

        color = colormap[idx, 0:3]
        # Set the open3d point cloud
        corners_in_camera = corners_in_camera_lst[idx]

        for i in range(0, corners_in_camera.shape[1]):  # iterate each corner
            x, y, z = corners_in_camera[0:3, i]
            draw_sphere(entities, x, y, z, radius=0.03, color=color)

    # Draw lidar corners
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):

        if 'intersections' not in annotations[collection_key]:
            continue

        for intersection_key, intersection in annotations[collection_key]['intersections'].items():
            intersection_point_in_lidar = np.asarray([intersection['x'],
                                                      intersection['y'],
                                                      intersection['z'], 1])

            # initial state before calibration
            intersection_point_in_camera = camera_T_lidar_initial @ intersection_point_in_lidar
            x, y, z = intersection_point_in_camera[0:3]

            color = colormap[idx, 0:3]*0.3  # use lidar cylinders darker to distinguish
            draw_cylinder(entities, x, y, z, radius=0.01, height=0.15, color=color)

            # after calibration
            intersection_point_in_camera = camera_T_lidar @ intersection_point_in_lidar
            x, y, z = intersection_point_in_camera[0:3]

            color = colormap[idx, 0:3]*0.7  # use lidar cylinders less darker to distinguish
            draw_cylinder(entities, x, y, z, radius=0.01, height=0.15, color=color)

    # Draw lidar coordinate frames per collection (should all be close to the same pose)
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
        collection_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        collection_frame = collection_frame.transform(camera_T_lidar_lst[idx])
        color = colormap[idx, 0:3]
        collection_frame.paint_uniform_color(color)
        entities.append(collection_frame)

    # Draw estimated joint lidar coordinate frame
    estimated_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    estimated_frame = estimated_frame.transform(camera_T_lidar)
    entities.append(estimated_frame)

    # Draw ground truth joint lidar coordinate frame
    ground_truth_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    ground_truth_frame = ground_truth_frame.transform(camera_T_lidar_gt)
    ground_truth_frame.paint_uniform_color((0.1, 0.1, 0.1))
    entities.append(ground_truth_frame)

    o3d.visualization.draw_geometries(entities,
                                      zoom=view['trajectory'][0]['zoom'],
                                      front=view['trajectory'][0]['front'],
                                      lookat=view['trajectory'][0]['lookat'],
                                      up=view['trajectory'][0]['up'])

    # TODO compare to gt and save file


if __name__ == "__main__":
    main()
