# Standard imports
import os
from datetime import datetime

# Ros imports
import rospkg
import atom_core
import tf

# Atom imports
from urdf_parser_py.urdf import Pose as URDFPose
from urdf_parser_py.urdf import URDF
from atom_core.config_io import execute, uriReader
from atom_core.naming import generateKey


def readXacroFile(description_file):
    # xml_robot = URDF.from_parameter_server()
    urdf_file = '/tmp/description.urdf'
    # print('Parsing description file ' + description_file)
    execute('xacro ' + description_file + ' -o ' + urdf_file, verbose=False)  # create a temp urdf file
    try:
        xml_robot = URDF.from_xml_file(urdf_file)  # read teh urdf file
    except:
        raise ValueError('Could not parse description file ' + description_file)

    return xml_robot


def saveResultsXacro(dataset, selected_collection_key, transforms_list):
    # Cycle all sensors in calibration config, and for each replace the optimized transform in the original xacro
    # Parse xacro description file
    description_file, _, _ = uriReader(dataset["calibration_config"]["description_file"])
    xml_robot = readXacroFile(description_file)

    for transform_key in transforms_list:

        # NOTE This is only valid if the generateTransformKey was called with argument separator='-'
        parent = transform_key.split('-')[0]
        child = transform_key.split('-')[1]

        trans = list(dataset["collections"][selected_collection_key]["transforms"][transform_key]["trans"])
        quat = list(dataset["collections"][selected_collection_key]["transforms"][transform_key]["quat"])
        found = False

        for joint in xml_robot.joints:
            if joint.parent == parent and joint.child == child:
                found = True

                # if origin is None create a new URDFPose.
                # See https://github.com/lardemua/atom/issues/559
                if joint.origin is None:  #
                    joint.origin = URDFPose()

                # print("Replacing xyz = " + str(joint.origin.xyz) + " by " + str(trans))
                joint.origin.xyz = trans

                rpy = list(tf.transformations.euler_from_quaternion(quat, axes="sxyz"))
                # print("Replacing rpy = " + str(joint.origin.rpy) + " by " + str(rpy))
                joint.origin.rpy = rpy
                break

        if not found:
            raise ValueError("Could not find transform " + str(transform_key) + " in " + description_file)

    time = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
    file_name = "optimized_" + time + ".urdf.xacro"

    # TODO we should assume all datasets have a _metadata field.
    if "_metadata" not in dataset:
        # if args['output_xacro'] is None:
        filename_results_xacro = "/tmp/" + file_name
        print("The dataset does not have the metadata field.")
        with open(filename_results_xacro, "w") as out:
            out.write(URDF.to_xml_string(xml_robot))
    else:
        # urdf_file, _, _ = atom_core.config_io.uriReader(dataset['calibration_config']['description_file'])
        # urdf_path = os.path.dirname(urdf_file)
        # path_to_file = urdf_path + '/optimized/'

        rospack = rospkg.RosPack()
        path_to_file = (rospack.get_path(dataset["_metadata"]["robot_name"] + "_calibration") + "/urdf/optimized/")

        if not os.path.exists(path_to_file):
            os.mkdir(path_to_file)
        filename_results_xacro = path_to_file + file_name
        with open(filename_results_xacro, "w") as out:
            out.write(URDF.to_xml_string(xml_robot))

        optimized_urdf_file = urdf_path + '/optimized.urdf.xacro'
        with open(optimized_urdf_file, "w", ) as out:
            out.write(URDF.to_xml_string(xml_robot))
        # print("Saving optimized.urdf.xacro in " + filename_results_xacro + ".")

    print("Optimized xacro saved to " + str(filename_results_xacro) + " . You can use it as a ROS robot_description.")
