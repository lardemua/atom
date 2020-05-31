#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------

import rospy
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
    rospy.init_node("create_initial_robot_description")
    rospy.sleep(0.1)

    xacro_file = rospy.get_param('~xacro_file')
    rospy.loginfo('xacro_file is ' + xacro_file)
    prefix = rospy.get_param('~prefix')
    rospy.loginfo('prefix is ' + prefix)

    # Parse robot description from param /robot_description
    rospy.loginfo('Reading xml xacro file ...')
    # xml_robot = URDF.from_xml_file(xacro_file)
    xml_robot = URDF.from_parameter_server()


    rospy.loginfo('Changing link names ...')
    for link in xml_robot.links:
        link.name = prefix + link.name

    rospy.loginfo('Changing joint names, parents and childs ...')
    for joint in xml_robot.joints:
        joint.name = prefix + joint.name
        joint.parent = prefix + joint.parent
        joint.child = prefix + joint.child

    # print(URDF.to_xml_string(xml_robot))
    rospy.loginfo('Writting new initial_robot_description parameter')
    rospy.set_param('initial_robot_description', URDF.to_xml_string(xml_robot))

