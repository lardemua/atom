#!/usr/bin/env python

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
counter = 0
h_first_entry = 0


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform((0, 0, sin(counter / 140.0) * 2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame")
    counter += 1

def processFeedback(feedback):
    global handle

    handle = feedback.menu_entry_id
    if handle == 1:
        print(feedback.marker_name)
        print("position: " + str(feedback.pose.position.x) + " , " + str(
            feedback.pose.position.y) + " , " + str(feedback.pose.position.z))
        print("orientation: " + str(feedback.pose.orientation.x) + " , " + str(
            feedback.pose.orientation.y) + " , " + str(feedback.pose.orientation.z))
        print(" in frame " + feedback.header.frame_id)

    menu_handler.reApply(server)

    server.applyChanges()

def alignMarker(feedback):
    pose = feedback.pose

    pose.position.x = round(pose.position.x - 0.5) + 0.5
    pose.position.y = round(pose.position.y - 0.5) + 0.5

    rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(
        feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                  str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))

    server.setPose(feedback.marker_name, pose)
    server.applyChanges()

def rand(min_, max_):
    return min_ + random() * (max_ - min_)

def makeBox(msg, r, g, b):
    marker = Marker()

    marker.type = Marker.CYLINDER
    marker.scale.x = msg.scale * 0.5
    marker.scale.y = msg.scale * 0.5
    marker.scale.z = msg.scale * 0.5
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0

    return marker

def makeBoxControl(msg, r, g, b):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg, r, g, b))
    msg.controls.append(control)
    return control

def make6DofMarker(interaction_mode, position, name, show_6dof = 0):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    #int_marker.pose.orientation = orientation
    int_marker.scale = 0.2

    int_marker.name = name
    int_marker.description = name

    # insert a box
    makeBoxControl(int_marker, 0, 1, 0)
    int_marker.controls[0].interaction_mode = interaction_mode

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)


def initMenu():
    global h_first_entry
    h_first_entry = menu_handler.insert("Position & Orientation", callback=processFeedback)

if __name__ == "__main__":
    rospy.init_node("basic_controls_with_menu")
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    robot_description = rospy.get_param('/robot_description')

    from urdf_parser_py.urdf import URDF

    robot = URDF.from_parameter_server()

    server = InteractiveMarkerServer("basic_controls_with_menu")
    count = 0

    # parsing of robot description
    for sensor in robot.sensors:
        ind = str(sensor.origin.xyz).index(', ')
        ind2 = str(sensor.origin.xyz)[(ind + 1):len(str(sensor.origin.xyz))].index(', ')
        x = str(sensor.origin.xyz)[1:ind]
        y = str(sensor.origin.xyz)[(ind + 2):(ind + ind2 + 1)]
        z = str(sensor.origin.xyz)[(ind + ind2 + 3):(len(str(sensor.origin.xyz)) - 1)]

        position = Point(float(x), float(y), float(z))
        count = count + 1
        make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, position, "Marker" + str(count), 1)


    print('Number of sensors: ' + str(count))

    initMenu()

    server.applyChanges()

    rospy.spin()

