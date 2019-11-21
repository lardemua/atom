#!/usr/bin/env python

import sys
import os.path
import argparse

from colorama import Fore, Back, Style

# ROS imports
import rospkg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler              import MenuHandler

from urdf_parser_py.urdf import URDF

# __self__ imports
from interactive_calibration.utilities import CalibConfig, validateLinks
from interactive_calibration.sensor    import Sensor

class InteractiveFirstGuess(object):

    def __init__(self, args):
        # command line arguments
        self.args = args
        # calibration config
        self.config = None
        # sensors
        self.sensors = []
        # robot URDF
        self.urdf = None
        # interactive markers server
        self.server = None
        self.menu   = None

    def init(self):
        # The parameter /robot_description must exist
        if not rospy.has_param('/robot_description'):
            rospy.logerr("The parameter does not '/robot_description' and it is required to continue")
            sys.exit(1)

        self.config = CalibConfig()
        ok = self.config.loadJSON(self.args['calibration_file'])
        # Exit if it fails to open a valid calibration file
        if not ok: sys.exit(1)

        # Load the urdf from /robot_description
        self.urdf = URDF.from_parameter_server()

        ok = validateLinks(self.config.world_link, self.config.sensors, self.urdf)
        if not ok: sys.exit(1)

        print('Number of sensors: ' + str(len(self.config.sensors)))

        # Init interaction
        self.server = InteractiveMarkerServer("interactive_first_guess")
        self.menu   = MenuHandler()

        self.menu.insert("Save sensors configuration",     callback=self.onSaveFirstGuess)
        self.menu.insert("Reset to initial configuration", callback=self.onReset)

        # For each node generate an interactive marker.
        for name, sensor in self.config.sensors.items():
            print(Fore.BLUE + '\nSensor name is ' + name + Style.RESET_ALL)
            params = {
                "frame_world":      self.config.world_link,
                "frame_opt_parent": sensor.parent_link,
                "frame_opt_child":  sensor.child_link,
                "frame_sensor":     sensor.link,
                "marker_scale":     self.args['marker_scale']}
            # Append to the list of sensors
            self.sensors.append(Sensor(sensor.name, self.server, self.menu, **params))

        self.server.applyChanges()

    def onSaveFirstGuess(self, feedback):
        for sensor in self.sensors:
            for joint in self.urdf.joints:  # find corresponding joint for this sensor
                if sensor.opt_child_link == joint.child and sensor.opt_parent_link == joint.parent:
                    trans = sensor.optT.getTranslation()
                    euler = sensor.optT.getEulerAngles()
                    joint.origin.xyz = list(trans)
                    joint.origin.rpy = list(euler)

        # Write the urdf file with interactive_calibration's source path as base directory.
        rospack = rospkg.RosPack()
        outfile = rospack.get_path('interactive_calibration') + os.path.abspath('/' + self.args['filename'])
        with open(outfile, 'w') as out:
            print("Writing fist guess urdf to '{}'".format(outfile))
            out.write(self.urdf.to_xml_string())

    def onReset(self, feedback):
        for sensor in self.sensors:
            sensor.resetToInitalPose()


if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filename", type=str, required=True, default="/calibrations/atlascar2/atlascar2_first_guess.urdf.xacro",
                    help="Full path and name of the first guess xacro file. Starting from the root of the interactive calibration ros package")
    ap.add_argument("-s", "--marker_scale",     type=float, default=0.6, help='Scale of the interactive markers.')
    ap.add_argument("-c", "--calibration_file", type=str, required=True, help='full path to calibration file.')

    args = vars(ap.parse_args())

    # Initialize ROS stuff
    rospy.init_node("sensors_first_guess")

    # Launch the application !!
    first_guess = InteractiveFirstGuess(args)
    first_guess.init()

    print('Changes applied ...')

    rospy.spin()

