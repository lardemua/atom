#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
from _elementtree import ElementTree

import matplotlib

from interactive_markers.interactive_marker_server import *
from matplotlib import cm
from urdf_parser_py.urdf import URDF
# from urdfdom_py.urdf import URDF
# import urdfdom_py
import rospkg
from Sensor import *
from colorama import Fore, Back, Style
from graphviz import Digraph

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

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--world_link", help='Name of the reference frame wich is common to all sensors. Usually '
                                               'it is the world or base_link.', type=str, required=True)
    args = vars(ap.parse_args())

    # Initialize ROS stuff
    rospy.init_node("draw_calibration_graph")
    rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
    robot_description = rospy.get_param('/robot_description')

    g = Digraph('G', filename='calibration_transformations.gv')

    # Parse robot description from param /robot_description
    xml_robot = URDF.from_parameter_server()
    # robot = URDF.from_xml_file(rospack.get_path('interactive_marker_test') + "/urdf/atlas_macro.urdf.xacro")

    # Process robot description and create an instance of class Sensor for each sensor
    number_of_sensors = 0
    sensors = []


    for joint in xml_robot.joints:
        if 'caster' in joint.name:
            continue
        g.node(joint.parent,_attributes={'shape': 'ellipse'})
        g.node(joint.child, _attributes={'shape': 'ellipse'})
        g.edge(joint.parent, joint.child, color='black',style='dashed')




    print('Number of sensors: ' + str(len(xml_robot.sensors)))
    # cmap = cm.Set3(np.linspace(0, 1, len(xml_robot.sensors)))
    cmap = cm.Pastel2(np.linspace(0, 1, len(xml_robot.sensors)))

    # parsing of robot description
    for i, xml_sensor in enumerate(xml_robot.sensors):

        print(Fore.BLUE + '\n\nSensor name is ' + xml_sensor.name + Style.RESET_ALL)

        # Check if we have all the information needed. Abort if not.
        if xml_sensor.parent is None:
            raise ValueError('Element parent for sensor ' + xml_sensor.name + ' must be specified in the urdf/xacro.')
        else:
            print('parent link is ' + str(xml_sensor.parent))

        if xml_sensor.calibration_parent is None:
            raise ValueError(
                'Element calibration_parent for sensor ' + xml_sensor.name + ' must be specified in the urdf/xacro.')
        else:
            print('calibration_parent is ' + str(xml_sensor.calibration_parent))

        if xml_sensor.calibration_child is None:
            raise ValueError(
                'Element calibration_child for sensor ' + xml_sensor.name + ' must be specified in the urdf/xacro.')
        else:
            print('calibration_child is ' + str(xml_sensor.calibration_child))


        rgb = matplotlib.colors.rgb2hex(cmap[i, 0:3])

        g.edge(args['world_link'], xml_sensor.calibration_parent, xml_sensor.name + '\\n(pre-transform)',
               color=rgb, style='solid', _attributes={'penwidth':'4'})

        g.edge(xml_sensor.calibration_parent, xml_sensor.calibration_child, xml_sensor.name + '\\n(to be calibrated)',
               color=rgb, style='solid', _attributes={'penwidth':'4'})

        g.edge(xml_sensor.calibration_child, xml_sensor.parent, xml_sensor.name + '\\n(post-transform)',
               color=rgb, style='solid', _attributes={'penwidth':'4'})


        g.node(xml_sensor.parent, xml_sensor.parent, _attributes={'penwidth': '4', 'color':rgb})

        # with g.subgraph(name='cluster_' + xml_sensor.name) as sg:
        #     addEdge(sg, xml_sensor.calibration_child, xml_sensor.parent, ' ' + xml_sensor.name + ' (post)')


    g.view()
    rospy.spin()
