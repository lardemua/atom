#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse


from interactive_markers.interactive_marker_server import *
from urdf_parser_py.urdf import URDF
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

    print('Number of sensors: ' + str(len(xml_robot.sensors)))

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


        def addEdge(g, parent, child, label, color='black'):
            if not parent == child:
                # g.attr('node', shape='box')
                g.node(parent, label, _attributes={'shape': 'ellipse'})
                g.node(child, label, _attributes={'shape': 'ellipse'})
                g.edge(parent, child, label=label, color=color, style='dashed')

        # Append to the list of sensors
        sensors.append(Sensor(xml_sensor.name, server, menu_handler, args['world_link'],
                              xml_sensor.calibration_parent, xml_sensor.calibration_child, xml_sensor.parent))

    initMenu()
    server.applyChanges()
    print('Changes applied ...')

    rospy.spin()
