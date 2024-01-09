"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

# Standard imports
import numpy as np
from scipy.spatial import distance

# ROS imports
from tf import transformations

# Atom imports
from atom_core.geometry import matrixToRodrigues, rodriguesToMatrix

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

# ------------  Sensors -----------------
# Each sensor will have a position (tx,ty,tz) and a rotation (r1,r2,r3)


def getterTransform(dataset, transform_key, collection_name):
    # The pose must be returned as a list of translation vector and rotation, i.e. [tx, ty, tz, r1, r2, r3] where r1,
    # r2,r3 are angles of the Rodrigues rotation vector.

    trans = dataset['collections'][collection_name]['transforms'][transform_key]['trans']
    quat = dataset['collections'][collection_name]['transforms'][transform_key]['quat']

    # Convert from (trans, quat) to [tx, ty, tz, r1, r2, r3] format
    h_matrix = transformations.quaternion_matrix(quat)  # quaternion to homogeneous matrix
    matrix = h_matrix[0:3, 0:3]  # non-homogeneous matrix 3x3
    rod = matrixToRodrigues(matrix).tolist()  # matrix to Rodrigues
    return trans + rod


def setterTransform(dataset, values, transform_key, collection_name=None):
    # The pose must be returned as a list of translation vector and rotation, i.e. [tx, ty, tz, r1, r2, r3] where r1,
    # r2,r3 are angles of the Rodrigues rotation vector.

    # Convert from [tx, ty, tz, r1, r2, r3] format to trans and quat format
    trans, rod = values[0:3], values[3:]
    matrix = rodriguesToMatrix(rod)
    h_matrix = np.identity(4)
    h_matrix[0:3, 0:3] = matrix
    quat = transformations.quaternion_from_matrix(h_matrix)

    if collection_name is None:  # if collection_name is None, set all collections with the same value
        for collection_key in dataset['collections']:
            dataset['collections'][collection_key]['transforms'][transform_key]['trans'] = trans  # set the translation
            dataset['collections'][collection_key]['transforms'][transform_key]['quat'] = quat  # set the quaternion
    else:
        dataset['collections'][collection_name]['transforms'][transform_key]['trans'] = trans  # set the translation
        dataset['collections'][collection_name]['transforms'][transform_key]['quat'] = quat  # set the quaternion


def getterSensorTranslation(data, sensor_key, collection_key):
    calibration_parent = data['sensors'][sensor_key]['calibration_parent']
    calibration_child = data['sensors'][sensor_key]['calibration_child']
    transform_key = calibration_parent + '-' + calibration_child
    # We use collection selected_collection and assume they are all the same
    return data['collections'][collection_key]['transforms'][transform_key]['trans']


def setterSensorTranslation(data, value, sensor_key):
    assert len(value) == 3, "value must be a list with length 3."

    calibration_parent = data['sensors'][sensor_key]['calibration_parent']
    calibration_child = data['sensors'][sensor_key]['calibration_child']
    transform_key = calibration_parent + '-' + calibration_child

    for _collection_key in data['collections']:
        data['collections'][_collection_key]['transforms'][transform_key]['trans'] = value


def getterSensorRotation(data, sensor_key, collection_key):
    calibration_parent = data['sensors'][sensor_key]['calibration_parent']
    calibration_child = data['sensors'][sensor_key]['calibration_child']
    transform_key = calibration_parent + '-' + calibration_child

    # We use collection selected_collection and assume they are all the same
    quat = data['collections'][collection_key]['transforms'][transform_key]['quat']
    hmatrix = transformations.quaternion_matrix(quat)
    matrix = hmatrix[0:3, 0:3]

    return matrixToRodrigues(matrix)


def setterSensorRotation(data, value, sensor_key):
    assert len(value) == 3, "value must be a list with length 3."

    matrix = rodriguesToMatrix(value)
    hmatrix = np.identity(4)
    hmatrix[0:3, 0:3] = matrix
    quat = transformations.quaternion_from_matrix(hmatrix)

    calibration_parent = data['sensors'][sensor_key]['calibration_parent']
    calibration_child = data['sensors'][sensor_key]['calibration_child']
    transform_key = calibration_parent + '-' + calibration_child

    for _collection_key in data['collections']:
        data['collections'][_collection_key]['transforms'][transform_key]['quat'] = quat


def getterCameraPMatrix(data, sensor_key):
    fx_p = data['sensors'][sensor_key]['camera_info']['P'][0]
    fy_p = data['sensors'][sensor_key]['camera_info']['P'][5]
    cx_p = data['sensors'][sensor_key]['camera_info']['P'][2]
    cy_p = data['sensors'][sensor_key]['camera_info']['P'][6]
    intrinsics = [fx_p, fy_p, cx_p, cy_p]
    return intrinsics


def setterCameraPMatrix(data, value, sensor_key):
    assert len(value) == 4, "value must be a list with length 4."
    data['sensors'][sensor_key]['camera_info']['P'][0] = value[0]  # fx_p
    data['sensors'][sensor_key]['camera_info']['P'][5] = value[1]  # fy_p
    data['sensors'][sensor_key]['camera_info']['P'][2] = value[2]  # cx_p
    data['sensors'][sensor_key]['camera_info']['P'][6] = value[3]  # cy_p


def getterCameraIntrinsics(data, sensor_key):
    fx = data['sensors'][sensor_key]['camera_info']['K'][0]
    fy = data['sensors'][sensor_key]['camera_info']['K'][4]
    cx = data['sensors'][sensor_key]['camera_info']['K'][2]
    cy = data['sensors'][sensor_key]['camera_info']['K'][5]
    D = data['sensors'][sensor_key]['camera_info']['D']
    intrinsics = [fx, fy, cx, cy]
    intrinsics.extend(D)
    return intrinsics


def setterCameraIntrinsics(data, value, sensor_key):
    assert len(value) == 9, "value must be a list with length 9."
    data['sensors'][sensor_key]['camera_info']['K'][0] = value[0]
    data['sensors'][sensor_key]['camera_info']['K'][4] = value[1]
    data['sensors'][sensor_key]['camera_info']['K'][2] = value[2]
    data['sensors'][sensor_key]['camera_info']['K'][5] = value[3]
    data['sensors'][sensor_key]['camera_info']['D'] = value[4:]


# ------------  Patterns -----------------
# Each sensor will have a position (tx,ty,tz) and a rotation (r1,r2,r3)

def getterPatternTranslation(data, collection_key):
    return data['patterns']['collections'][collection_key]['trans']


def setterPatternTranslation(data, value, collection_key):
    assert len(value) == 3, "value must be a list with length 3."
    data['patterns']['collections'][collection_key]['trans'] = value


def getterPatternRotation(data, collection_key):
    quat = data['patterns']['collections'][collection_key]['quat']
    hmatrix = transformations.quaternion_matrix(quat)
    matrix = hmatrix[0:3, 0:3]
    return matrixToRodrigues(matrix)


def setterPatternRotation(data, value, collection_key):
    assert len(value) == 3, "value must be a list with length 3."

    matrix = rodriguesToMatrix(value)
    hmatrix = np.identity(4).astype(float)
    hmatrix[0:3, 0:3] = matrix
    quat = transformations.quaternion_from_matrix(hmatrix)
    data['patterns']['collections'][collection_key]['quat'] = quat


def getterJointParam(dataset, joint_key, param_key, collection_name):
    return [dataset['collections'][collection_name]['joints'][joint_key][param_key]]


def setterJointParam(dataset, value, joint_key, param_key, collection_name):
    if collection_name is None:  # if collection_name is None, set all collections with the same value
        for collection_key in dataset['collections']:
            dataset['collections'][collection_key]['joints'][joint_key][param_key] = value[0]
    else:
        dataset['collections'][collection_name]['joints'][joint_key][param_key] = value
