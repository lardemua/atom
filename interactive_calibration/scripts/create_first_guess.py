#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import MenuHandler
from urdf_parser_py.urdf import URDF
import rospkg
# from AtlasCarCalibration.interactive_calibration.src.Sensor import *
from colorama import Fore, Back, Style

import interactive_calibration.sensor

# from interactive_calibration import D
from graphviz import Digraph
import json

# from interactive_calibration import Tra

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
        filename = rospack.get_path('interactive_calibration') + args['filename']
        f = open(filename, "w")
        f.write(xml_string)
        f.close()
        print('Saved first guess to file ' + filename)
    if handle == 2:
        for sensor in sensors:
            interactive_calibration.sensor.Sensor.resetToInitalPose(sensor)


def initMenu():
    menu_handler.insert("Save sensors configuration", callback=menuFeedback)
    menu_handler.insert("Reset to initial configuration", callback=menuFeedback)


if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--world_link", help='Name of the reference frame wich is common to all sensors. Usually '
                                               'it is the world or base_link.', type=str, required=True)
    ap.add_argument("-f", "--filename", help="Full path and name of the first guess xacro file. Starting from the root of the interactive calibration ros package",
                    type=str, required=True, default="/calibrations/atlascar2/atlascar2_first_guess.urdf.xacro")
    ap.add_argument("-s", "--marker_scale", help='Scale of the interactive markers.', type=float, default=0.6)
    ap.add_argument("-c", "--calibration_file", help='full path to calibration file.', type=str, required=True)
    args = vars(ap.parse_args())

    # Initialize ROS stuff
    rospy.init_node("sensors_first_guess")
    rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
    server = InteractiveMarkerServer("sensors_first_guess")
    robot_description = rospy.get_param('/robot_description')
    rospy.sleep(0.5)

    # Parse robot description from param /robot_description
    robot_from_json = json.load(open(args['calibration_file'], 'r'))

    # Process robot description and create an instance of class Sensor for each sensor
    number_of_sensors = 0
    sensors = []

    print('Number of sensors: ' + str(len(robot_from_json)))

    # parsing of robot description
    # for i, xml_sensor in enumerate(xml_robot.sensors):
    for key, value in robot_from_json.items():

        print(Fore.BLUE + '\n\nSensor name is ' + key + Style.RESET_ALL)

        #TODO put this in a function and adapt for the json case
        # # Check if we have all the information needed. Abort if not.
        # for attr in ['parent', 'calibration_parent', 'calibration_child']:
        #     if not hasattr(xml_sensor, attr):
        #         raise ValueError(
        #             'Element ' + attr + ' for sensor ' + xml_sensor.name + ' must be specified in the urdf/xacro.')

        # Append to the list of sensors
        sensors.append(interactive_calibration.sensor.Sensor(key, server, menu_handler, args['world_link'],
                                                             value['calibration_parent_link'],
                                                             value['calibration_child_link'], value['sensor_link'],
                                                             marker_scale=args['marker_scale']))

    initMenu()
    server.applyChanges()
    print('Changes applied ...')

    rospy.spin()
