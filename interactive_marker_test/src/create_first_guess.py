#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import matplotlib

# import network as nx
from interactive_markers.interactive_marker_server import *
from matplotlib import cm
from urdf_parser_py.urdf import URDF
import rospkg
from Sensor import *
from colorama import Fore, Back, Style
import matplotlib.pyplot as plt  # Library to do plots 2D
# from graphviz import Digraph

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------

server = None
menu_handler = MenuHandler()


# ------------------------
#      FUNCTIONS         #
# ------------------------

def menuFeedback(feedback):
    # print('called menu')
    handle = feedback.menu_entry_id
    # Update
    if handle == 1:
        for sensor in sensors:
            for joint in xml_robot.joints:  # find corresponding joint for this sensor
                if sensor.opt_child_link == joint.child and sensor.opt_parent_link == joint.parent:
                    trans = sensor.optT.getTranslation()
                    euler = sensor.optT.getEulerAngles()
                    joint.origin.xyz[0] = trans[0]
                    joint.origin.xyz[1] = trans[1]
                    joint.origin.xyz[2] = trans[2]
                    joint.origin.rpy[0] = euler[0]
                    joint.origin.rpy[1] = euler[1]
                    joint.origin.rpy[2] = euler[2]

        xml_string = xml_robot.to_xml_string()
        filename = rospack.get_path('interactive_marker_test') + "/urdf/macro_first_guess.urdf.xacro"
        f = open(filename, "w")
        f.write(xml_string)
        f.close()
        print('Saved first guess to file ' + filename)
    if handle == 2:
        for sensor in sensors:
            Sensor.resetToInitalPose(sensor)


def initMenu():
    menu_handler.insert("Save sensors configuration", callback=menuFeedback)
    menu_handler.insert("Reset to initial configuration", callback=menuFeedback)


# from graphviz import Digraph
#
# g = Digraph('G', filename='cluster_edge.gv')
# # g.attr(compound='true')
#
# with g.subgraph(name='cluster0') as c:
#     c.edges(['ab', 'ac', 'bd', 'cd'])
#
# # with g.subgraph(name='cluster1') as c:
# #     c.edges(['eg', 'ef'])
#
# # g.edge('b', 'f', lhead='cluster1')
# # g.edge('d', 'e')
# # g.edge('c', 'g', ltail='cluster0', lhead='cluster1')
# # g.edge('c', 'e', ltail='cluster0')
# # g.edge('d', 'h')
#
# g.view()
# exit(0)

if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--world_link", help='Name of the reference frame wich is common to all sensors. Usually '
                                               'it is the world or base_link.', type=str, required=True)
    args = vars(ap.parse_args())

    # Initialize ROS stuff
    rospy.init_node("sensors_first_guess")
    rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
    server = InteractiveMarkerServer("sensors_first_guess")
    robot_description = rospy.get_param('/robot_description')
    rospy.sleep(0.5)

    # Parse robot description from param /robot_description
    xml_robot = URDF.from_parameter_server()
    # robot = URDF.from_xml_file(rospack.get_path('interactive_marker_test') + "/urdf/atlas_macro.urdf.xacro")

    # Process robot description and create an instance of class Sensor for each sensor
    number_of_sensors = 0
    sensors = []

    # g = Digraph('G', filename='calibration_transformations.gv')

    print('Number of sensors: ' + str(len(xml_robot.sensors)))
    # cmap = cm.Set3(np.linspace(0, 1, len(xml_robot.sensors)))

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


        # def addEdge(g, parent, child, label, color='black'):
        #     if not parent == child:
        #         g.edge(parent, child, label=label, color=color, style='dashed')


        # rgb = cmap[i, 0:3]
        # # node_attr={'shape': 'box', 'color': 'blue'}
        # addEdge(g, args['world_link'], xml_sensor.calibration_parent, ' Pre-transform\\n' + xml_sensor.name, 'grey')
        # addEdge(g, xml_sensor.calibration_parent, xml_sensor.calibration_child,
        #         ' Calibration\\n' + xml_sensor.name, color=matplotlib.colors.rgb2hex(rgb))
        # addEdge(g, xml_sensor.calibration_child, xml_sensor.parent, ' Post-transform\\n' + xml_sensor.name,
        #         color=matplotlib.colors.rgb2hex(rgb))
        # with g.subgraph(name='cluster_' + xml_sensor.name) as sg:
        #     addEdge(sg, xml_sensor.calibration_child, xml_sensor.parent, ' ' + xml_sensor.name + ' (post)')

        #
        #
        # g.edge('b', 'f', lhead=xml_sensor.name)

        # Append to the list of sensors
        sensors.append(Sensor(xml_sensor.name, server, menu_handler, args['world_link'],
                              xml_sensor.calibration_parent, xml_sensor.calibration_child, xml_sensor.parent))

    initMenu()
    server.applyChanges()
    print('Changes applied ...')

    # g.view()
    rospy.spin()
