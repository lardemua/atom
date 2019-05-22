#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import numpy as np
import matplotlib
import rospy
from matplotlib import cm
from tf import TransformListener
from urdf_parser_py.urdf import URDF
import rospkg
from colorama import Fore, Style
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

    # Parse robot description from param /robot_description
    xml_robot = URDF.from_parameter_server()
    # robot = URDF.from_xml_file(rospack.get_path('interactive_calibration') + "/urdf/atlas_macro.urdf.xacro")

    # ----------------------------------------------
    # Create a full transformation graph annotated
    # ----------------------------------------------
    g = Digraph('G', filename='calibration_full')
    number_of_sensors = 0
    sensors = []

    for joint in xml_robot.joints:
        if 'caster' in joint.name:
            continue
        g.node(joint.parent, _attributes={'shape': 'ellipse'})
        g.node(joint.child, _attributes={'shape': 'ellipse'})
        g.edge(joint.parent, joint.child, color='black', style='dashed')

    print('Number of sensors: ' + str(len(xml_robot.sensors)))
    # cmap = cm.Set3(np.linspace(0, 1, len(xml_robot.sensors)))
    cmap = cm.Pastel2(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.viridis(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.brg(np.linspace(0, 1, len(xml_robot.sensors)))

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
               color=rgb, style='solid', _attributes={'penwidth': '2'})

        g.edge(xml_sensor.calibration_parent, xml_sensor.calibration_child, xml_sensor.name + '\\n(to be calibrated)',
               color=rgb, style='solid', _attributes={'penwidth': '2'})

        g.edge(xml_sensor.calibration_child, xml_sensor.parent, xml_sensor.name + '\\n(post-transform)',
               color=rgb, style='solid', _attributes={'penwidth': '2'})
        g.node(xml_sensor.parent, xml_sensor.parent, _attributes={'penwidth': '4', 'color': rgb})

    g.view()

    # ----------------------------------------------
    # Draw transform chain for each Sensor
    # ----------------------------------------------
    listener = TransformListener()
    rospy.sleep(2.0)
    g2 = Digraph('G', filename='calibration_per_sensor')

    for i, xml_sensor in enumerate(xml_robot.sensors):

        print(Fore.BLUE + '\n\nSensor name is ' + xml_sensor.name + Style.RESET_ALL)

        rgb = matplotlib.colors.rgb2hex(cmap[i, 0:3])

        chain = listener.chain(xml_sensor.parent, rospy.Time(), args['world_link'], rospy.Time(), args['world_link'])
        print('Complete chain is: ' + str(chain))

        for parent, child in zip(chain[0::], chain[1::]):
            print(parent + '->' + child)
            if parent == xml_sensor.calibration_parent and child == xml_sensor.calibration_child:
                g2.edge(parent, child, color=rgb, style='solid', _attributes={'penwidth': '1', 'fontcolor':rgb},
                    label=xml_sensor.name + '\\n calibration transform')
            else:
                g2.edge(parent, child, color=rgb, style='solid', _attributes={'penwidth': '1'})

    # get the type of this joint: fixed, revolute, prismatic, etc
    for i, joint in enumerate(xml_robot.joints):
        if joint.type == 'fixed':
            g2.edge(joint.parent, joint.child, color='black', style='solid', _attributes={'penwidth': '1', 'fontcolor':'black'},
                    label=joint.type + ' joint')
        else:
            g2.edge(joint.parent, joint.child, color='black', style='solid', _attributes={'penwidth': '1', 'fontcolor':'black'},
                    label=joint.type + ' joint')


    for i, xml_sensor in enumerate(xml_robot.sensors):
        rgb = matplotlib.colors.rgb2hex(cmap[i, 0:3])
        g2.node(xml_sensor.parent, xml_sensor.parent, _attributes={'penwidth': '4', 'color': rgb})

    g2.view()
