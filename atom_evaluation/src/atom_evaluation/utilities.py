"""
ATOM evaluation utilities
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import numpy as np
from atom_core.atom import getTransform


def atomicTfFromCalibration(dataset, anchored_sensor_key, other_sensor_key, calib_tf):
    # ----------------------------------------------------------------------------------------------------------------
    # ------ Get input data: required links and transformation
    # ----------------------------------------------------------------------------------------------------------------
    world_link = dataset['calibration_config']['world_link']
    anchored_sensor_link = dataset['calibration_config']['sensors'][anchored_sensor_key]['link']
    other_sensor_link = dataset['calibration_config']['sensors'][other_sensor_key]['link']
    other_sensor_child_link = dataset['calibration_config']['sensors'][other_sensor_key]['child_link']
    other_sensor_parent_link = dataset['calibration_config']['sensors'][other_sensor_key]['parent_link']

    selected_collection_key = dataset['collections'].keys()[0]
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
