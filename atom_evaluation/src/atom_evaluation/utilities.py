"""
ATOM evaluation utilities
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import numpy as np
import math
from atom_core.atom import getTransform


def atomicTfFromCalibration(dataset, anchored_sensor_key, other_sensor_key, calib_tf, anchored_additional_data=False,
                            other_additional_data=False):
    if anchored_additional_data:
        anchored_type = 'additional_data'
    else:
        anchored_type = 'sensors'

    if other_additional_data:
        other_type = 'additional_data'
    else:
        other_type = 'sensors'

    # ----------------------------------------------------------------------------------------------------------------
    # ------ Get input data: required links and transformation
    # ----------------------------------------------------------------------------------------------------------------
    world_link = dataset['calibration_config']['world_link']
    anchored_sensor_link = dataset['calibration_config'][anchored_type][anchored_sensor_key]['link']
    other_sensor_link = dataset['calibration_config'][other_type][other_sensor_key]['link']
    other_sensor_child_link = dataset['calibration_config'][other_type][other_sensor_key]['child_link']
    other_sensor_parent_link = dataset['calibration_config'][other_type][other_sensor_key]['parent_link']

    selected_collection_key = list(dataset['collections'].keys())[0]
    base2anchored = getTransform(world_link, anchored_sensor_link,
                                 dataset['collections'][selected_collection_key]['transforms'])
    base2other_parent = getTransform(world_link, other_sensor_parent_link,
                                     dataset['collections'][selected_collection_key]['transforms'])
    other_child2other = getTransform(other_sensor_child_link, other_sensor_link,
                                     dataset['collections'][selected_collection_key]['transforms'])

    Ta = np.dot(np.linalg.inv(base2other_parent), base2anchored)
    Tb = np.dot(Ta, calib_tf)
    Tc = np.dot(Tb, np.linalg.inv(other_child2other))

    return Tc


def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. Euclidean norm, along axis.
    """
    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data * data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data


def quaternion_slerp(quat0, quat1, fraction, spin=0, shortestpath=True):
    """Return spherical linear interpolation between two quaternions.
    """
    _EPS = np.finfo(float).eps * 4.0
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    if fraction == 0.0:
        return q0
    elif fraction == 1.0:
        return q1
    d = np.dot(q0, q1)
    if abs(abs(d) - 1.0) < _EPS:
        return q0
    if shortestpath and d < 0.0:
        # invert rotation
        d = -d
        np.negative(q1, q1)
    angle = math.acos(d) + spin * math.pi
    if abs(angle) < _EPS:
        return q0
    isin = 1.0 / math.sin(angle)
    q0 *= math.sin((1.0 - fraction) * angle) * isin
    q1 *= math.sin(fraction * angle) * isin
    q0 += q1
    return q0


def random_quaternion(rand=None):
    """Return uniform random unit quaternion.
    rand: array like or None
        Three independent random variables that are uniformly distributed
        between 0 and 1.
    """
    if rand is None:
        rand = np.random.rand(3)
    else:
        assert len(rand) == 3
    r1 = np.sqrt(1.0 - rand[0])
    r2 = np.sqrt(rand[0])
    pi2 = math.pi * 2.0
    t1 = pi2 * rand[1]
    t2 = pi2 * rand[2]
    return np.array([np.cos(t2) * r2, np.sin(t1) * r1,
                     np.cos(t1) * r1, np.sin(t2) * r2])


def averageTransforms(l_transforms):
    N = len(l_transforms)
    print("Computing the average of " + str(N) + " transforms")

    # Get a list of all the translations l_t
    l_t = [i[0] for i in l_transforms]
    # print(l_t)

    # compute the average translation
    tmp1 = sum(v[0] for v in l_t) / N
    tmp2 = sum(v[1] for v in l_t) / N
    tmp3 = sum(v[2] for v in l_t) / N
    avg_t = (tmp1, tmp2, tmp3)

    # Get a list of the rotation quaternions
    l_q = [i[1] for i in l_transforms]

    # Average the quaterions using an incremental slerp approach
    acc = 1.0  # accumulator, counts the number of observations inserted already
    avg_q = random_quaternion()  # can be random for start, since it will not be used
    for q in l_q:
        # How to deduce the ratio on each iteration
        # w_q = 1 / (acc + 1)
        # w_qavg = acc  / (acc + 1)
        # ratio = w_q / w_qavg <=> 1 / acc
        avg_q = quaternion_slerp(avg_q, q, 1.0 / acc)  # run pairwise slerp
        acc = acc + 1  # increment acc

    avg_q = tuple(avg_q)

    return (avg_t, avg_q)
