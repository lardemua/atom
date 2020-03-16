#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import json
import os
import rospkg

import rospy
import tf
from urdf_parser_py.urdf import URDF

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------

# ------------------------
#      FUNCTIONS         #
# ------------------------

if __name__ == "__main__":
    # Initialize ROS stuff
    rospy.init_node("generate_xacro_from_results")
    rospy.sleep(0.1)

    # ---------------------------------------
    # --- Parse command line argument
    # ---------------------------------------
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    args = vars(ap.parse_args())
    print("\nArgument list=" + str(args) + '\n')

    """ Loads a json file containing the detections"""
    f = open(args['json_file'], 'r')
    dataset_results = json.load(f)

    # xacro_file = rospy.get_param('~xacro_file')
    xacro_file = "/home/mike/catkin_ws/src/atlascar2/atlascar2_description/urdf/atlascar2.urdf.xacro"
    rospy.loginfo('xacro_file is ' + xacro_file)

    # Parse robot description from param /robot_description
    rospy.loginfo('Reading xml xacro file: ' + xacro_file)
    # xml_robot = URDF.from_xml_file(xacro_file)
    xml_robot = URDF.from_parameter_server()
    # print(xml_robot)

    # For the getters we only need to get one collection. Lets take the first key on the dictionary and always get that
    # transformation.
    selected_collection_key = dataset_results['collections'].keys()[0]

    for sensor_key in dataset_results['calibration_config']['sensors']:
        child = dataset_results['calibration_config']['sensors'][sensor_key]['child_link']
        parent = dataset_results['calibration_config']['sensors'][sensor_key]['parent_link']
        transform_key = parent + '-' + child

        trans = list(dataset_results['collections'][selected_collection_key]['transforms'][transform_key]['trans'])
        quat = list(dataset_results['collections'][selected_collection_key]['transforms'][transform_key]['quat'])

        for joint in xml_robot.joints:
            if joint.parent == parent and joint.child == child:
                print('Found joint: ' + str(joint.name))

                print('Replacing xyz = ' + str(joint.origin.xyz) + ' by ' + str(trans))
                joint.origin.xyz = trans

                rpy = list(tf.transformations.euler_from_quaternion(quat, axes='rxyz'))
                print('Replacing rpy = ' + str(joint.origin.rpy) + ' by ' + str(rpy))
                joint.origin.rpy = rpy


    print(URDF.to_xml_string(xml_robot))

    rospack = rospkg.RosPack()
    outfile = rospack.get_path('interactive_calibration') + '/calibrations/atlascar2/optimized.urdf.xacro'
    with open(outfile, 'w') as out:
        print("Writing optimized urdf to " + str(outfile))
        out.write(URDF.to_xml_string(xml_robot))
    #
    # rospy.loginfo('Changing link names ...')
    # for link in xml_robot.links:
    #     print('link is ' + link.name)

    # rospy.loginfo('Changing joint names, parents and childs ...')
    # for joint in xml_robot.joints:
    #     joint.name = prefix + joint.name
    #     joint.parent = prefix + joint.parent
    #     joint.child = prefix + joint.child

    # print(URDF.to_xml_string(xml_robot))
    rospy.loginfo('Writting results xacro ... ')
    # rospy.set_param('initial_robot_description', URDF.to_xml_string(xml_robot))
