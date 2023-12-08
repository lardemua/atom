# Standard imports
import os
from datetime import datetime
import sys
from colorama import Fore, Style
from copy import deepcopy

# Ros imports
import rospkg
import yaml
import atom_core
from atom_core.utilities import atomError
from atom_core.xacro_io import readXacroFile
import tf

# Atom imports
from urdf_parser_py.urdf import Pose as URDFPose
from urdf_parser_py.urdf import URDF, Link, Joint, Mesh, Visual, JointCalibration
from atom_core.system import execute
from atom_core.config_io import uriReader
from atom_core.naming import generateKey


def saveResultsYml(dataset, selected_collection_key, output_file):

    dict_yml = {}  # The dictionary containing all calibrated parameters and their optimized values

    # Cycle all sensors in calibration config, and for each replace the optimized transform in the original xacro
    # Parse xacro description file
    for sensor_key, sensor in dataset["calibration_config"]['sensors'].items():

        # Search for corresponding transform. Since this is a sensor transformation it must be static, which is why we use only one collection, the selected collection key
        found = False
        for transform_key, transform in dataset['collections'][selected_collection_key]['transforms'].items():

            if sensor['parent_link'] == transform['parent'] and sensor['child_link'] == transform['child']:
                trans = transform['trans']
                quat = transform['quat']
                rpy = tf.transformations.euler_from_quaternion(quat, axes='rxyz')

                dict_yml[sensor_key + '_x'] = trans[0]
                dict_yml[sensor_key + '_y'] = trans[1]
                dict_yml[sensor_key + '_z'] = trans[2]

                dict_yml[sensor_key + '_roll'] = rpy[0]
                dict_yml[sensor_key + '_pitch'] = rpy[1]
                dict_yml[sensor_key + '_yaw'] = rpy[2]
                found = True
                break

        if not found:
            raise atomError("Could not find transform for sensor " + sensor_key +
                            '. Cannot produce yaml file with calibration results.')

    # Cycle all additional_tfs in calibration config, and for each replace the optimized transform in the original xacro
    # Parse xacro description file
    if dataset['calibration_config']['additional_tfs'] is not None:
        for additional_tf_key, additional_tf in dataset["calibration_config"]['additional_tfs'].items():

            # Search for corresponding transform. Since this is a sensor transformation it must be static, which is why we use only one collection, the selected collection key
            found = False
            for transform_key, transform in dataset['collections'][selected_collection_key]['transforms'].items():

                if additional_tf['parent_link'] == transform['parent'] and additional_tf['child_link'] == transform['child']:
                    trans = transform['trans']
                    quat = transform['quat']
                    rpy = tf.transformations.euler_from_quaternion(quat, axes='rxyz')

                    dict_yml[sensor_key + '_x'] = trans[0]
                    dict_yml[sensor_key + '_y'] = trans[1]
                    dict_yml[sensor_key + '_z'] = trans[2]

                    dict_yml[sensor_key + '_roll'] = rpy[0]
                    dict_yml[sensor_key + '_pitch'] = rpy[1]
                    dict_yml[sensor_key + '_yaw'] = rpy[2]
                    found = True
                    break

            if not found:
                raise atomError("Could not find transform for additional_tf " + additional_tf_key +
                                '. Cannot produce yaml file with calibration results.')

    # Cycle all joints in calibration config, and for each replace the optimized transform in the original xacro
    # Parse xacro description file
    if dataset['calibration_config']['joints'] is not None:
        for config_joint_key, config_joint in dataset["calibration_config"]['joints'].items():

            # Search for corresponding transform. Since this is a sensor transformation it must be static, which is why we use only one collection, the selected collection key
            found = False
            for joint_key, joint in dataset['collections'][selected_collection_key]['joints'].items():

                if config_joint_key == joint_key:

                    for param_to_calibrate in config_joint['params_to_calibrate']:
                        calibrated_value = dataset['collections'][selected_collection_key]['joints'][joint_key][param_to_calibrate]
                        dict_yml[config_joint_key + '_' + param_to_calibrate] = calibrated_value

                    found = True
                    break

            if not found:
                raise atomError("Could not find transform for additional_tf " + additional_tf_key +
                                '. Cannot produce yaml file with calibration results.')

    # Make sure all values in dict are float
    # https://stackoverflow.com/questions/21695705/dump-an-python-object-as-yaml-file
    for key in dict_yml.keys():
        dict_yml[key] = float(dict_yml[key])

    # DEBUG
    # for key in dict_yml.keys():  # just to verify
    #     print(key + ' is of type ' + str(type(dict_yml[key])))
    # print(dict_yml)

    print('Saved calibrated parameters to yaml file ' + Fore.BLUE + output_file + Style.RESET_ALL)
    yaml.dump(dict_yml, open(output_file, 'w'))
