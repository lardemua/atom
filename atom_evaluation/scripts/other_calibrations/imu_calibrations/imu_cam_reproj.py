#!/usr/bin/env python3

"""
Consider two collections in a dataset: A and B. Consider also a robotic system with a rigid camera-IMU transform. 
The idea is to calculate the reprojection error of the camera in collection B by integrating the IMU data from A to B to get the final pose of the camera.
"""

import argparse
import pprint
from copy import deepcopy
import sys
from typing import List, Tuple, Dict
import numpy as np
from scipy.spatial.transform import Rotation

from copy import deepcopy
from atom_calibration.calibration.derivation.derivation_utils import timeStampToFloat
from atom_core.atom import getTransform
from atom_core.dataset_io import (
    copyTFToDataset,
    filterPatternsFromDataset,
    getMixedDataset,
    loadResultsJSON,
    filterCollectionsFromDataset,
    filterSensorsFromDataset,
)
from atom_core.drawing import drawCross2D, drawSquare2D
from atom_core.utilities import (
    atomStartupPrint,
    createLambdaExpressionsForArgs,
    saveFileResults,
    atomError,
)
from atom_core.vision import projectToCamera
from numpy._typing import NDArray


def getPointsInPatternAsNPArray(_collection_key, _pattern_key, _sensor_key, _dataset):
    pts_in_pattern_list = []  # collect the points
    for pt_detected in _dataset["collections"][_collection_key]["labels"][_pattern_key][
        _sensor_key
    ]["idxs"]:
        id_detected = pt_detected["id"]
        point = [
            item
            for item in _dataset["patterns"][_pattern_key]["corners"]
            if item["id"] == id_detected
        ][0]
        pts_in_pattern_list.append(point)

    return np.array(
        [
            [item["x"] for item in pts_in_pattern_list],  # convert list to np array
            [item["y"] for item in pts_in_pattern_list],
            [0 for _ in pts_in_pattern_list],
            [1 for _ in pts_in_pattern_list],
        ],
        float,
    )


def skew(vec):
    skew = np.array(
        [
            [0, -vec[0], -vec[1], -vec[2]],
            [vec[0], 0, vec[2], -vec[1]],
            [vec[1], -vec[2], 0, vec[0]],
            [vec[2], vec[1], -vec[0], 0],
        ]
    )

    return skew


def getIMUData(
    dataset: Dict, imu_sensor_name: str, start_time: float, end_time: float
) -> List:

    imu_topic: str = dataset["sensors"][imu_sensor_name]["topic"]
    imu_continuous_data_list: List = dataset["continuous_data"][imu_topic]

    imu_continuous_data_start_to_end = list(
        filter(
            lambda x: timeStampToFloat(x["header"]["stamp"]) >= start_time
            and timeStampToFloat(x["header"]["stamp"]) <= end_time,
            imu_continuous_data_list,
        )
    )

    return imu_continuous_data_start_to_end


def integrate(
    imu_data: List, start_quat: NDArray, start_pos: NDArray, ignore_gravity: bool
) -> Tuple:

    # quaternion structure is x,y,z,w
    quat_current = start_quat
    pos_current = start_pos
    lin_vel_current = np.array([0, 0, 0])

    for i in range(len(imu_data) - 1):
        # To do RK4 integration, we need, for each step:
        #     - initial quaternion
        #     - dt between steps
        #     - angular velocities on both the current and next datapoint

        dt = timeStampToFloat(imu_data[i + 1]["header"]["stamp"]) - timeStampToFloat(
            imu_data[i]["header"]["stamp"]
        )
        ang_vel_current = np.array(
            [
                imu_data[i]["angular_velocity"]["x"],
                imu_data[i]["angular_velocity"]["y"],
                imu_data[i]["angular_velocity"]["z"],
            ]
        )
        lin_accel_current = np.array(
            [
                imu_data[i]["linear_acceleration"]["x"],
                imu_data[i]["linear_acceleration"]["y"],
                imu_data[i]["linear_acceleration"]["z"],
            ]
        )
        ang_vel_next = np.array(
            [
                imu_data[i + 1]["angular_velocity"]["x"],
                imu_data[i + 1]["angular_velocity"]["y"],
                imu_data[i + 1]["angular_velocity"]["z"],
            ]
        )
        lin_accel_next = np.array(
            [
                imu_data[i + 1]["linear_acceleration"]["x"],
                imu_data[i + 1]["linear_acceleration"]["y"],
                imu_data[i + 1]["linear_acceleration"]["z"],
            ]
        )

        q1 = quat_current
        k1 = 0.5 * skew(ang_vel_current) @ q1

        q2 = quat_current + dt * (0.5 * k1)
        k2 = 0.5 * skew((ang_vel_current + ang_vel_next) / 2) @ q2

        q3 = quat_current + dt * (0.5 * k2)
        k3 = 0.5 * skew((ang_vel_current + ang_vel_next) / 2) @ q3

        q4 = quat_current + dt * k3
        k4 = 0.5 * skew(ang_vel_next) @ q4

        new_quat = quat_current + dt * (k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6)
        new_quat = new_quat / np.linalg.norm(new_quat)

        # Now integrate translation
        r_current = Rotation.from_quat(quat_current)
        r_next = Rotation.from_quat(new_quat)

        lin_accel_current = r_current.as_matrix() @ lin_accel_current
        lin_accel_next = r_next.as_matrix() @ lin_accel_next

        if not ignore_gravity:
            lin_accel_current[2] -= 9.81
            lin_accel_next[2] -= 9.81

        lin_vel_next = lin_vel_current + ((lin_accel_current + lin_accel_next) * dt / 2)

        # Velocity correction
        delta_s = lin_vel_next / (i + 2)
        lin_vel_next_corrected = lin_vel_next - delta_s

        pos_next = pos_current + ((lin_vel_current + lin_vel_next_corrected) * dt / 2)

        quat_current = new_quat
        lin_vel_current = lin_vel_next_corrected
        pos_current = pos_next

    return quat_current, pos_current


def main() -> None:

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-json",
        "--json_file",
        help="Json file containing input dataset.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-csf",
        "--collection_selection_function",
        default=None,
        type=str,
        help="A string to be evaluated into a lambda function that receives a collection name as input and "
        "returns True or False to indicate if the collection should be loaded (and used in the "
        "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
        "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.",
    )
    # save results in a csv file
    ap.add_argument(
        "-sfr",
        "--save_file_results",
        help="Store the results",
        action="store_true",
        default=False,
    )
    ap.add_argument(
        "-sfrn",
        "--save_file_results_name",
        help="Name of csv file to save the results. "
        "Default: -test_json/results/{name_of_dataset}_{sensor_source}_to_{sensor_target}_results.csv",
        type=str,
        required=False,
    )
    ap.add_argument(
        "-uic",
        "--use_incomplete_collections",
        action="store_true",
        default=False,
        help="Remove any collection which does not have a detection for all sensors.",
    )
    ap.add_argument(
        "-rpd",
        "--remove_partial_detections",
        help="Remove detected labels which are only partial." "Used or the Charuco.",
        action="store_true",
        default=False,
    )
    ap.add_argument(
        "-ssf",
        "--sensor_selection_function",
        default=None,
        type=str,
        help="a string to be evaluated into a lambda function that receives a sensor name as input and "
        "returns true or false to indicate if the sensor should be loaded (and used in the "
        "optimization). the syntax is lambda name: f(x), where f(x) is the function in python "
        'language. example: lambda name: name in ["left_laser", "frontal_camera"] , to load only '
        "sensors left_laser and frontal_camera",
    )
    ap.add_argument(
        "-imu",
        "--imu_sensor_name",
        help="The name of the IMU link.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-c", "--camera", help="Camera sensor name.", type=str, required=True
    )
    ap.add_argument(
        "-p", "--pattern", help="Calibration pattern name.", type=str, required=True
    )

    # - Save args
    # args = vars(ap.parse_known_args()[0])
    arglist = [x for x in sys.argv[1:] if not x.startswith("__")]
    # these args have the selection functions as strings
    args_original = vars(ap.parse_args(args=arglist))
    args = createLambdaExpressionsForArgs(
        args_original
    )  # selection functions are now lambdas

    imu_sensor_name = args["imu_sensor_name"]
    camera_sensor_name = args["camera"]

    # ---------------------------------------
    # --- INITIALIZATION Read calibration data from file
    # ---------------------------------------
    # Loads the train json file containing the calibration results
    original_dataset, json_file = loadResultsJSON(
        args["json_file"], args["collection_selection_function"]
    )

    dataset = filterCollectionsFromDataset(original_dataset, args)

    # GET collection pairs
    collection_pairs: List[Tuple] = []

    collection_keys = list(dataset["collections"].keys())
    for i in range(len(collection_keys)):
        # ignore first collection
        if i == 0:
            continue

        collection_pair = (collection_keys[i - 1], collection_keys[i])
        collection_pairs.append(collection_pair)

    # Get world frame
    world_link = dataset["calibration_config"]["world_link"]

    # Create error dictionary. Each collection pair will have a corresponding error.
    e = {}
    # Now calculate error for each collection pair
    for collection_pair in collection_pairs:
        start_collection = collection_pair[0]
        start_time = timeStampToFloat(
            dataset["collections"][start_collection]["data"][imu_sensor_name]["header"][
                "stamp"
            ]
        )

        end_collection = collection_pair[1]
        end_time = timeStampToFloat(
            dataset["collections"][end_collection]["data"][imu_sensor_name]["header"][
                "stamp"
            ]
        )

        # Get world-camera pose in start collection
        start_tf_pool = dataset["collections"][start_collection]["transforms"]
        start_world_cam_tf = getTransform(
            from_frame=world_link,
            to_frame=dataset["sensors"][camera_sensor_name]["parent"],
            transforms=start_tf_pool,
        )
        start_world_imu_tf = getTransform(
            from_frame=world_link,
            to_frame=dataset["sensors"][imu_sensor_name]["parent"],
            transforms=start_tf_pool,
        )

        # Now get IMU data from the start collection until end collection
        imu_data = getIMUData(
            dataset=dataset,
            imu_sensor_name=imu_sensor_name,
            start_time=start_time,
            end_time=end_time,
        )

        imu_R = Rotation.from_matrix(start_world_imu_tf[:3, :3])
        start_imu_quat = imu_R.as_quat()
        start_imu_pos = start_world_imu_tf[:3, 3].T

        end_quat, end_pos = integrate(
            imu_data=imu_data,
            start_quat=start_imu_quat,
            start_pos=start_imu_pos,
            ignore_gravity=False,
        )

        # For ease of use
        start_collection_key = collection_pair[0]
        end_collection_key = collection_pair[1]
        end_collection = deepcopy(dataset["collections"][end_collection_key])

        # Replace end tf to end_collection object
        pprint.pp(end_collection["transforms"])
        exit(0)

        # Check if collection B has label information
        if "labels" not in end_collection:
            print(
                f"Collection {end_collection_key} does not have labels information. Skipping..."
            )
            continue

        # Create error dict for A-B collection pair
        e[f"{start_collection_key}-{end_collection_key}"] = {}

        for pattern_key, pattern in dataset["calibration_config"][
            "calibration_patterns"
        ].items():
            e[f"{start_collection_key}-{end_collection_key}"][pattern_key] = {}

            # Get number of pattern corners
            nx = dataset["calibration_config"]["calibration_patterns"][pattern_key][
                "dimension"
            ]["x"]
            ny = dataset["calibration_config"]["calibration_patterns"][pattern_key][
                "dimension"
            ]["y"]

            # Check if pattern is detected by camera
            if not end_collection["labels"][pattern_key][camera_sensor_name][
                "detected"
            ]:
                continue

            # Get the pattern corners in the local pattern frame. Must use only corners which have -----------------
            # correspondence to the detected points stored in collection['labels'][sensor_key]['idxs'] -------------
            pts_in_pattern = getPointsInPatternAsNPArray(
                end_collection_key, pattern_key, camera_sensor_name, dataset
            )

            # Transform the pts from the pattern's reference frame to the sensor's reference frame -----------------
            from_frame = dataset["sensors"][camera_sensor_name]["parent"]
            to_frame = dataset["calibration_config"]["calibration_patterns"][
                pattern_key
            ]["link"]
            sensor_to_pattern = getTransform(
                from_frame, to_frame, end_collection["transforms"]
            )
            pts_in_sensor = np.dot(sensor_to_pattern, pts_in_pattern)

            # Project points to the image of the sensor ------------------------------------------------------------
            w, h = (
                end_collection["data"][camera_sensor_name]["width"],
                end_collection["data"][camera_sensor_name]["height"],
            )
            sensor = dataset["sensors"][camera_sensor_name]
            K = np.ndarray(
                (3, 3), buffer=np.array(sensor["camera_info"]["K"]), dtype=float
            )
            D = np.ndarray(
                (5, 1), buffer=np.array(sensor["camera_info"]["D"]), dtype=float
            )

            pts_in_image, _, _ = projectToCamera(K, D, w, h, pts_in_sensor[0:3, :])


if __name__ == "__main__":
    main()
