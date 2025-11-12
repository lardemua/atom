#!/usr/bin/env python3

import argparse
import numpy as np
from numpy._typing import NDArray
from atom_core.geometry import matrixToTranslationRotation
from atom_core.dataset_io import (
    addNoiseToInitialGuess,
    filterCollectionsFromDataset,
    loadResultsJSON,
)
import math
from atom_core.transformations import compareTransforms
from atom_core.atom import getTransform


def getKalibrTF(file_name: str) -> NDArray:

    file = open(file_name, "r")

    res_index = None
    for index, line in enumerate(file):

        if "T_ci" in line:
            res_index = [index + j for j in range(1, 5)]
            res_lines = []

        if res_index and index in res_index:
            res_lines.append(line)

    file.close()

    for idx in range(len(res_lines)):
        res_lines[idx] = res_lines[idx].replace("\n", "")
        res_lines[idx] = res_lines[idx].replace("[", "")
        res_lines[idx] = res_lines[idx].replace("]", "")

        res_lines[idx] = res_lines[idx].split(" ")

        res_lines[idx] = [float(x) for x in res_lines[idx] if x != ""]

    kalibr_cam_imu_tf = np.array(res_lines)

    return kalibr_cam_imu_tf


def getAtomTf(file_name: str, imu_sensor: str, camera_sensor: str) -> NDArray:

    # Read dataset file
    dataset, json_file = loadResultsJSON(file_name, None)  # 2nd arg is csf
    selected_collection_key = list(dataset["collections"].keys())[0]

    tf_pool = dataset["collections"][selected_collection_key]["transforms"]

    source_frame = dataset["sensors"][camera_sensor]["calibration_child"]
    # source_frame = "hand_rgb"
    target_frame = dataset["sensors"][imu_sensor]["calibration_child"]

    res = getTransform(
        from_frame=source_frame,
        to_frame=target_frame,
        transforms=tf_pool,
    )

    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-json",
        "--json_file",
        help="Json file containing ATOM dataset.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-krf",
        "--kalibr_results_file",
        help=".txt file containing kalibr cam-IMU calibration results.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-imu",
        "--imu_sensor",
        help="IMU sensor name.",
        type=str,
        required=True,
    )
    ap.add_argument(
        "-c",
        "--camera",
        help="Camera sensor name.",
        type=str,
        required=True,
    )

    # - Save args
    args = vars(ap.parse_known_args()[0])

    kalibr_res = getKalibrTF(args["kalibr_results_file"])

    gt = getAtomTf(
        file_name=args["json_file"],
        imu_sensor=args["imu_sensor"],
        camera_sensor=args["camera"],
    )
    translation_error, rotation_error, _, _, _, _, _, _ = compareTransforms(
        kalibr_res, gt
    )

    print("Etrans = " + str(round(translation_error * 1000, 3)) + " (mm)")
    print("Erot = " + str(round(rotation_error * 180 / math.pi, 3)) + " (deg)")


if __name__ == "__main__":
    main()
