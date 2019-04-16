#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

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
marker_pos = 0
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

    # parsing of robot description
    server = InteractiveMarkerServer("basic_controls_with_menu")

    start = 0
    count = 0
    countc = 0
    num_mark = 0
    startc = 0
    ind = robot_description.find('<link name="marker', start, len(robot_description))
    ind_joint = robot_description.find('<joint name="', ind, len(robot_description))

    ind_pos1 = robot_description.find('xyz="', ind, len(robot_description))
    first_space = robot_description.find(' ', ind_pos1 + 1, len(robot_description))
    second_space = robot_description.find(' ', first_space + 1, len(robot_description))
    last_space = robot_description.find('"', second_space + 1, len(robot_description))

    ind_pos2 = robot_description.find('xyz="', ind_joint, len(robot_description))
    first_space2 = robot_description.find(' ', ind_pos2 + 1, len(robot_description))
    second_space2 = robot_description.find(' ', first_space2 + 1, len(robot_description))
    last_space2 = robot_description.find('"', second_space2 + 1, len(robot_description))

    indc = robot_description.find('<!--<link name="marker', start, len(robot_description))

    if ind == indc + 4:
        start = start + ind
    elif ind != indc:
        count = count + 1

        position = Point(float(robot_description[(ind_pos1 + 5): first_space]) + float(robot_description[(ind_pos2 + 5):
                                                                                                  first_space2]),
                         float(robot_description[(first_space + 1): second_space]) + float(robot_description[(first_space2
                                                                                                       + 1): second_space2]),
                         float(robot_description[(second_space + 1): last_space]) + float(robot_description[(
                                                                                                             second_space2 + 1): last_space2]))
        make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, position, "Marker" + str(count), 1)

    while start < len(robot_description):

        if robot_description.find('<link name="marker', start, len(robot_description)) != ind:
            ind = robot_description.find('<link name="marker', start, len(robot_description))
            ind_joint = robot_description.find('<joint name="', ind, len(robot_description))

            ind_pos1 = robot_description.find('xyz="', ind, len(robot_description))
            first_space = robot_description.find(' ', ind_pos1 + 1, len(robot_description))
            second_space = robot_description.find(' ', first_space + 1, len(robot_description))
            last_space = robot_description.find('"', second_space + 1, len(robot_description))

            ind_pos2 = robot_description.find('xyz="', ind_joint, len(robot_description))
            first_space2 = robot_description.find(' ', ind_pos2 + 1, len(robot_description))
            second_space2 = robot_description.find(' ', first_space2 + 1, len(robot_description))
            last_space2 = robot_description.find('"', second_space2 + 1, len(robot_description))

            indc = robot_description.find('<!--<link name="marker', start, len(robot_description))
            if ind == indc + 4:
                start = ind
            elif ind != indc:
                count = count + 1

                position = Point(
                    float(robot_description[(ind_pos1 + 5): first_space]) + float(robot_description[(ind_pos2 + 5):
                                                                                                    first_space2]),
                    float(robot_description[(first_space + 1): second_space]) + float(robot_description[(first_space2
                                                                                                         + 1): second_space2]),
                    float(robot_description[(second_space + 1): last_space]) + float(robot_description[(
                                                                                                               second_space2 + 1): last_space2]))
                make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, position, "Marker" + str(count), 1)

        start = start + 1
    print ('Numero de marcadores: ' + str(count))


    initMenu()

    server.applyChanges()

    rospy.spin()

