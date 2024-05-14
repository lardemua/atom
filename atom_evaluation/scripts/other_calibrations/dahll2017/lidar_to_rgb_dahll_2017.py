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

    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # We only need to get one collection because optimized transformations are static, which means they are the same for all collections. Let's select the first key in the dictionary and always get that transformation.
    selected_collection_key = list(dataset["collections"].keys())[0]
    print("Selected collection key is " + str(selected_collection_key))

    # ---------------------------------------
    # Verifications
    # ---------------------------------------

    # ------------------------------------------
    # Set the ground truth initial pose
    # ------------------------------------------
    camera_T_lidar_gt = getTransform(
        from_frame=dataset["calibration_config"]["sensors"][args["camera"]]["link"],
        to_frame=dataset["calibration_config"]["sensors"][args["lidar"]]["link"],
        transforms=dataset_ground_truth["collections"][selected_collection_key]["transforms"],
    )

    # rot = Rotation.from_matrix(T[0:3, 0:3])
    # w, p, r = rot.as_euler("xyz", degrees=False)
    # x, y, z = T[0:3, 3]
    # y += 0.22
    # w += 129230498.03
    # p -= 0.14

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
        print(pts_in_camera)
        print(pts_in_camera.shape)

        # Top left corner
        x = 0 - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                      ]['size'] - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                                                                                        ]['border_size']['x']
        y = 0 - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                      ]['size'] - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                                                                                        ]['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 0] = pt_in_camera

        # Top right corner
        x = nx * dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                       ]['size'] + dataset['calibration_config']['calibration_patterns'][args['pattern']]['border_size']['x']
        y = 0 - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                      ]['size'] - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                                                                                        ]['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 1] = pt_in_camera

        # Bottom right corner
        x = nx * dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                       ]['size'] + dataset['calibration_config']['calibration_patterns'][args['pattern']]['border_size']['x']
        y = ny * dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                       ]['size'] + dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                                                                                         ]['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 2] = pt_in_camera

        # Bottom left corner
        x = 0 - dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                      ]['size'] - dataset['calibration_config']['calibration_patterns'][args['pattern']]['border_size']['x']
        y = ny * dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                       ]['size'] + dataset['calibration_config']['calibration_patterns'][args['pattern']
                                                                                                                                         ]['border_size']['y']
        z = 0

        pt_in_pattern = np.asarray([x, y, z, 1]).T
        pt_in_camera = camera_T_pattern @ pt_in_pattern
        pts_in_camera[:, 2] = pt_in_camera

        # append ot corners
        corners_in_camera_lst.append(pts_in_camera)

    # ------------------------------------------
    # Define visualization
    # ------------------------------------------
    entities = []
    colormap = cm.Set3(np.linspace(0, 1, len(dataset["collections"].keys())))

    camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    entities.append(camera_frame)

    # Add pattern frames
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
        pattern_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        pattern_frame = pattern_frame.transform(camera_T_pattern_lst[idx])
        entities.append(pattern_frame)

    # Draw lidar points (using ground truth transform)
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
        # Get the 3D LiDAR labelled points for the given collection
        points_in_lidar = getPointsInSensorAsNPArray(collection_key, args["pattern"],
                                                     args["lidar"], "idxs", dataset)

        # Transform points from lidar to camera coordinate frame
        points_in_camera = camera_T_lidar_gt @ points_in_lidar

        # Set the open3d point cloud
        pc_in_camera = o3d.geometry.PointCloud()
        pc_in_camera.points = o3d.utility.Vector3dVector(points_in_camera[0:3, :].transpose())

        color = colormap[idx, 0:3]
        pc_in_camera.paint_uniform_color(color)

        entities.append(pc_in_camera)

    # Draw camera corners
    for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):

        color = colormap[idx, 0:3]
        # Set the open3d point cloud
        pts = corners_in_camera_lst[idx]

        for i in range(0, pts.shape[1]):  # iterate each corner
            x, y, z = pts[0:3, i]
            draw_sphere(entities, x, y, z, radius=0.03, color=color)

    o3d.visualization.draw_geometries(entities,
                                      zoom=view['trajectory'][0]['zoom'],
                                      front=view['trajectory'][0]['front'],
                                      lookat=view['trajectory'][0]['lookat'],
                                      up=view['trajectory'][0]['up'])

    # vis.add_geometry(data['lidar_frame'])

    # for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
    #     vis.add_geometry(data['collections'][collection_key]['plane_frame'])

    # for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
    # vis.add_geometry(data['collections'][collection_key]['pc_in_camera'])

    # Cannot set view control as this only works in version 0.18
    # https://github.com/isl-org/Open3D/issues/6009
    # control = vis.get_view_control()
    # control.change_field_of_view(view['trajectory'][0]['field_of_view'])
    # control.set_front(view['trajectory'][0]['front'])
    # control.set_lookat(view['trajectory'][0]['lookat'])
    # control.set_up(view['trajectory'][0]['up'])
    # control.set_zoom(view['trajectory'][0]['zoom']+2000)

    # vis.poll_events()
    vis.update_renderer()

    def visualizationFunction(models):  # ------------------------

        data = models["data"]

        # Get transform in the 4x4 matrix format
        camera_T_lidar = get4x4TransformFromXYZWPR(data["transform"])

        data['lidar_frame'].vertices = data['lidar_frame_initial'].vertices
        data['lidar_frame'].transform(camera_T_lidar)
        vis.update_geometry(data['lidar_frame'])

        for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
            color = colormap[idx, 0:3]
            data['collections'][collection_key]['pc_in_camera'].paint_uniform_color(color)
            vis.update_geometry(data['collections'][collection_key]['pc_in_camera'])

        for idx, (collection_key, collection) in enumerate(dataset["collections"].items()):
            color = colormap[idx, 0:3]

            # Get the plane_T_camera transformation
            #      nx   ny   nz   t
            # T = [nx1  ny1  nz1  tx]
            # T = [nx2  ny2  nz2  ty]
            # T = [nx3  ny3  nz3  tz]
            # T = [  0    0    0   1]

            # Compute nz
            # We already have it from the plane detection
            nz = np.asarray([data['collections'][collection_key]['plane']['a'],
                             data['collections'][collection_key]['plane']['b'],
                             data['collections'][collection_key]['plane']['c']])

            # nx vector is arbitrary. Lets use the center point and
            # find another point that belongs to the plane
            x0 = data['collections'][collection_key]['center']['x']
            y0 = data['collections'][collection_key]['center']['y']
            z0 = data['collections'][collection_key]['center']['z']

            # compute new points xyz1 using plane equation
            a = data['collections'][collection_key]['plane']['a']
            b = data['collections'][collection_key]['plane']['b']
            c = data['collections'][collection_key]['plane']['c']
            d = data['collections'][collection_key]['plane']['d']

            x1 = x0 + 1  # arbitrary
            y1 = y0
            z1 = -(a * x1 + b * y1 + d) / c  # compute
            nx = np.asarray([x1 - x0, y1 - y0, z1 - z0])
            nx = normalize(nx)

            # ny vector is obtained from the cross product nz x nx
            ny = np.cross(nz, nx)
            ny = normalize(ny)

            # Compute camera_T_plane transform
            camera_T_plane = np.eye(4)
            camera_T_plane[0:3, 0] = nx
            camera_T_plane[0:3, 1] = ny
            camera_T_plane[0:3, 2] = nz
            camera_T_plane[0:3, 3] = [x0, y0, z0]  # translation

            data['collections'][collection_key]['plane_frame'].vertices = data['collections'][
                collection_key]['plane_frame_initial'].vertices
            data['collections'][collection_key]['plane_frame'].transform(camera_T_plane)
            vis.update_geometry(data['collections'][collection_key]['plane_frame'])

        #     data["collections"][collection_key]["pc_in_camera"].paint_uniform_color(color)
        #     entities.append(data["collections"][collection_key]["pc_in_camera"])

        #     # Draw points in camera

        #     np_center = data["collections"][collection_key]["pc_in_camera"].get_center()
        #     x0, y0, z0 = np_center[0], np_center[1], np_center[2]

        #     # Get plane equation
        #     a, b, c = nz[0], nz[1], nz[2]
        #     d = -a * np_center[0] - b * np_center[1] - c * np_center[2]

        #     # nx vector is arbitrary. Lets use the center point and
        #     # find another point that belongs to the plane
        #     x1 = x0 + 1
        #     y1 = y0
        #     z1 = -(a * x1 + b * y1 + d) / c
        #     nx = np.asarray([x1 - x0, y1 - y0, z1 - z0])
        #     nx = normalize(nx)

        #     # ny vector is obtained from the cross product nz x nx
        #     ny = np.cross(nz, nx)
        #     ny = normalize(ny)

        #     camera_T_pc_center = np.eye(4)
        #     camera_T_pc_center[0:3, 0] = nx
        #     camera_T_pc_center[0:3, 1] = ny
        #     camera_T_pc_center[0:3, 2] = nz
        #     camera_T_pc_center[0:3, 3] = np_center  # translation

        #     # Draw plane
        #     dx = 1.5
        #     dy = 1.0
        #     plane_mesh = o3d.geometry.TriangleMesh.create_box(
        #         width=dx, height=dy, depth=0.001
        #     )

        #     plane_mesh = plane_mesh.translate((-dx / 2, -dy / 2, 0))
        #     plane_mesh = plane_mesh.transform(camera_T_pc_center)

        #     plane_mesh_wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(
        #         plane_mesh
        #     )
        #     plane_mesh_wireframe.paint_uniform_color(color)
        #     entities.append(plane_mesh_wireframe)

        #     plane_axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        #     plane_axis = plane_axis.transform(camera_T_pc_center)
        #     entities.append(plane_axis)

        #     print(points_in_camera.shape)
        #     draw_sphere(
        #         entities,
        #         points_in_camera[0, 0],
        #         points_in_camera[1, 0],
        #         points_in_camera[2, 0],
        #         color=(0, 0.5, 0.5),
        #     )

        #     o3d.visualization.draw_geometries(
        #         entities,
        #         zoom=view["trajectory"][0]["zoom"],
        #         front=view["trajectory"][0]["front"],
        #         lookat=view["trajectory"][0]["lookat"],
        #         up=view["trajectory"][0]["up"],
        #         mesh_show_wireframe=False,
        #     )

        while not data['move_to_next_iteration']:
            vis.poll_events()
            vis.update_renderer()
        data['move_to_next_iteration'] = False

    opt.setVisualizationFunction(
        visualizationFunction, always_visualize=True, niterations=1
    )
    opt.setInternalVisualization(False)

    # ------------------------------------------
    # Sparse matrix
    # ------------------------------------------
    opt.computeSparseMatrix()
    opt.printSparseMatrix()

    # ------------------------------------------
    # Execution
    # ------------------------------------------
    # opt.callObjectiveFunction()
    # visualizationFunction(opt.data_models)

    opt.startOptimization(
        optimization_options={
            "x_scale": "jac",
            "ftol": 1e-4,
            "xtol": 1e-4,
            "gtol": 1e-4,
            "diff_step": None,
            "loss": "linear",
        }
    )

    print('Optimization finished. Press "q" to advance.')

    data["show_open3d"] = True
    opt.callObjectiveFunction()


if __name__ == "__main__":
    main()
