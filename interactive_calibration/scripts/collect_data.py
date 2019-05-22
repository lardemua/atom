#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import MenuHandler
from urdf_parser_py.urdf import URDF
import rospkg

# from AtlasCarCalibration.interactive_calibration.src.interactive_calibration.DataCollector import DataCollector
# from AtlasCarCalibration.interactive_calibration.src.interactive_calibration.sensor import *
import interactive_calibration.data_collector

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------
from visualization_msgs.msg import InteractiveMarkerControl, Marker

server = None
menu_handler = MenuHandler()


# ------------------------
#      FUNCTIONS         #
# ------------------------

def menuFeedback(feedback):
    print('Menu feedback')
    handle = feedback.menu_entry_id
    if handle == 1:  # collect snapshot
        print('Collect snapshot selected')
        data_collector.collectSnapshot()


def initMenu():
    menu_handler.insert("Collect snapshot", callback=menuFeedback)


def createInteractiveMarker():
    marker = InteractiveMarker()
    marker.header.frame_id = args['world_link']
    trans = (1,0,1)
    marker.pose.position.x = trans[0]
    marker.pose.position.y = trans[1]
    marker.pose.position.z = trans[2]
    quat = (0,0,0,1)
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale = 0.2

    marker.name = 'menu'
    marker.description = 'menu'

    # insert a box
    control = InteractiveMarkerControl()
    control.always_visible = True

    marker_box = Marker()
    marker_box.type = Marker.SPHERE
    marker_box.scale.x = marker.scale * 0.7
    marker_box.scale.y = marker.scale * 0.7
    marker_box.scale.z = marker.scale * 0.7
    marker_box.color.r = 0
    marker_box.color.g = 1
    marker_box.color.b = 0
    marker_box.color.a = 0.2

    control.markers.append(marker_box)
    marker.controls.append(control)

    marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    marker.controls.append(control)

    server.insert(marker, markerFeedback)
    menu_handler.apply(server, marker.name)



def markerFeedback(feedback):
    print('Received feedback')


if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument('-w', '--world_link', help='Name of the reference frame wich is common to all sensors. Usually '
                                               'it is the world or base_link.', type=str, required=True)
    ap.add_argument('-o', '--output_folder', help='Output folder to where the collected data will be stored.', type=str, required=True)
    args = vars(ap.parse_args())

    # Initialize ROS stuff
    rospy.init_node("sensors_first_guess")
    rospack = rospkg.RosPack()  # get an instance of RosPack with the default search paths
    server = InteractiveMarkerServer("sensors_first_guess")
    robot_description = rospy.get_param('/robot_description')
    rospy.sleep(0.5)

    # Parse robot description from param /robot_description
    xml_robot = URDF.from_parameter_server()
    # robot = URDF.from_xml_file(rospack.get_path('interactive_calibration') + "/urdf/atlas_macro.urdf.xacro")

    # Process robot description and create an instance of class Sensor for each sensor
    number_of_sensors = 0
    sensors = []

    print('Number of sensors: ' + str(len(xml_robot.sensors)))
    data_collector = interactive_calibration.data_collector.DataCollector(args['world_link'], args['output_folder'])

    createInteractiveMarker()
    initMenu()
    menu_handler.reApply(server)
    server.applyChanges()
    print('Changes applied ...')

    rospy.spin()
