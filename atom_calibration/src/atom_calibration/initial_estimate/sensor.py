#!/usr/bin/env python

# stdlib
import copy

# 3rd-party
import atom_msgs.srv
import colorama
import numpy as np
import cv2
import std_srvs.srv
import tf

from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from tf.listener import TransformListener
# from transformation_t import TransformationT
from atom_calibration.initial_estimate.transformation_t import TransformationT


# ------------------------
#      BASE CLASSES      #
# ------------------------

class MarkerPoseC:
    def __init__(self, position, orientation, frame_id, child_frame_id):
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

    def __str__(self):
        return str(self.position) + "\n" + str(self.orientation)

    __repr__ = __str__


class Sensor:

    def __init__(self, name, server, global_menu_handler, frame_world, frame_opt_parent, frame_opt_child, frame_sensor,
                 marker_scale):
        print('Creating a new sensor named ' + name)
        self.name = name
        self.visible = True
        self.server = server
        # self.global_menu_handler = global_menu_handler

        self.menu_handler = MenuHandler()
        self.listener = TransformListener()
        self.br = tf.TransformBroadcaster()
        self.marker_scale = marker_scale
        # transforms
        self.world_link = frame_world
        self.opt_parent_link = frame_opt_parent
        self.opt_child_link = frame_opt_child
        self.sensor_link = frame_sensor

        print('Collecting transforms...')
        self.updateAll()  # update all the transformations
        print('Collected pre, opt and pos transforms.')
        #
        # print('preT:\n' + str(self.preT))
        # print('optT:\n' + str(self.optT))
        # print('posT:\n' + str(self.posT))

        self.optTInitial = copy.deepcopy(self.optT)
        self.createInteractiveMarker()  # create interactive marker
        print('Created interactive marker.')

        # Add service to make visible / invisible and set the scale
        self.service_set_visible = rospy.Service('~' + self.name + '/set_sensor_interactive_marker',
                                                 atom_msgs.srv.SetSensorInteractiveMarker,
                                                 self.callbackSetSensorInteractiveMarker)

        # Add service to get the visible and scale
        self.service_get_visible = rospy.Service('~' + self.name + '/get_sensor_interactive_marker',
                                                 atom_msgs.srv.GetSensorInteractiveMarker,
                                                 self.callbackGetSensorInteractiveMarker)

        # Start publishing now
        self.timer_callback = rospy.Timer(rospy.Duration(.1), self.publishTFCallback)  # to periodically broadcast

    def callbackGetSensorInteractiveMarker(self, request):

        interactive_marker = self.server.get(self.name)
        response = atom_msgs.srv.GetSensorInteractiveMarkerResponse()
        response.visible = self.visible
        response.scale = interactive_marker.scale
        return response

    def callbackSetSensorInteractiveMarker(self, request):

        print('callbackSetSensorInteractiveMarker service requested for sensor ' + colorama.Fore.BLUE +
              self.name + colorama.Style.RESET_ALL)

        interactive_marker = self.server.get(self.name)
        interactive_marker.scale = request.scale
        self.visible = request.visible  # store visible state

        for control in interactive_marker.controls:
            if self.visible == 1:
                if 'move' in control.name:
                    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                elif 'rotate' in control.name:
                    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.NONE

        self.server.insert(interactive_marker)

        self.server.applyChanges()

        response = atom_msgs.srv.SetSensorInteractiveMarkerResponse()
        response.success = 1
        response.message = 'Control changed.'
        return response

    def resetToInitalPose(self, feedback=None):
        print('resetToInitialPose called for sensor ' + colorama.Fore.BLUE + self.name + colorama.Style.RESET_ALL)
        self.optT.matrix = self.optTInitial.matrix
        print('matrix=\n' + str(self.optT.matrix))

        trans = self.optT.getTranslation()
        self.marker.pose.position.x = trans[0]
        self.marker.pose.position.y = trans[1]
        self.marker.pose.position.z = trans[2]
        quat = self.optT.getQuaternion()
        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]

        self.optTInitial = copy.deepcopy(self.optT)

        self.updateMarkers()
        # self.global_menu_handler.reApply(self.server)
        self.server.applyChanges()

    def publishTFCallback(self, _):
        trans = self.optT.getTranslation()
        quat = self.optT.getQuaternion()
        self.br.sendTransform((trans[0], trans[1], trans[2]),
                              (quat[0], quat[1], quat[2], quat[3]),
                              rospy.Time.now(), self.opt_child_link, self.opt_parent_link)

    def markerFeedback(self, feedback):
        # print(' sensor ' + self.name + ' received feedback')

        self.optT.setTranslationFromPosePosition(feedback.pose.position)
        self.optT.setQuaternionFromPoseQuaternion(feedback.pose.orientation)

        self.updateMarkers()

        # self.global_menu_handler.reApply(self.server)
        self.server.applyChanges()

    def updateAll(self):
        self.updatePreT()
        self.updateOptT()
        self.updatePosT()

    def updateOptT(self):
        self.optT = self.updateT(self.opt_parent_link, self.opt_child_link, rospy.Time.now())

    def updatePreT(self):
        self.preT = self.updateT(self.world_link, self.opt_parent_link, rospy.Time.now())

    def updatePosT(self):
        self.posT = self.updateT(self.opt_child_link, self.sensor_link, rospy.Time.now())

    def updateT(self, parent_link, child_link, stamp):
        try:
            self.listener.waitForTransform(parent_link, child_link, rospy.Time(), rospy.Duration(1.0))
            (trans, quat) = self.listener.lookupTransform(parent_link, child_link, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            raise ValueError('Could not get transform from ' + parent_link + ' to ' + child_link + '(max 1 secs)')

        T = TransformationT(parent_link, child_link)
        T.setTranslation(trans)
        T.setQuaternion(quat)
        return T

    def updateMarkers(self):

        trans = self.optT.getTranslation()
        quat = self.optT.getQuaternion()

        marker = self.server.get(self.name)
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        self.server.insert(marker)

        marker = self.server.get(self.name + "_menu")
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        self.server.insert(marker)

        self.server.applyChanges()

    def createInteractiveMarker(self):
        self.marker = InteractiveMarker()
        self.marker.header.frame_id = self.opt_parent_link
        trans = self.optT.getTranslation()
        self.marker.pose.position.x = trans[0]
        self.marker.pose.position.y = trans[1]
        self.marker.pose.position.z = trans[2]
        quat = self.optT.getQuaternion()
        self.marker.pose.orientation.x = quat[0]
        self.marker.pose.orientation.y = quat[1]
        self.marker.pose.orientation.z = quat[2]
        self.marker.pose.orientation.w = quat[3]
        self.marker.scale = self.marker_scale

        self.marker.name = self.name
        self.marker.description = self.name + '_control'


        # insert a box
        control = InteractiveMarkerControl()
        control.always_visible = True

        marker_box = Marker()
        marker_box.type = Marker.SPHERE
        marker_box.scale.x = self.marker.scale * 0.3
        marker_box.scale.y = self.marker.scale * 0.3
        marker_box.scale.z = self.marker.scale * 0.3
        marker_box.color.r = 0
        marker_box.color.g = 1
        marker_box.color.b = 0
        marker_box.color.a = 0.2

        control.markers.append(marker_box)
        self.marker.controls.append(control)

        self.marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        self.marker.controls.append(control)

        self.server.insert(self.marker, self.markerFeedback)

        # Menu interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.opt_parent_link

        int_marker.pose.position.x = trans[0]
        int_marker.pose.position.y = trans[1]
        int_marker.pose.position.z = trans[2]

        int_marker.pose.orientation.x = quat[0]
        int_marker.pose.orientation.y = quat[1]
        int_marker.pose.orientation.z = quat[2]
        int_marker.pose.orientation.w = quat[3]
        int_marker.scale = self.marker_scale

        int_marker.name = self.name + "_menu"
        int_marker.description = "Right click()"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description = "-"
        control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        # make one control showing a sphere
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.marker_scale * 0.35
        marker.scale.y = self.marker_scale * 0.35
        marker.scale.z = self.marker_scale * 0.35
        marker_box.color.r = 0
        marker_box.color.g = 1
        marker_box.color.b = 0
        marker_box.color.a = 0.2
        control.markers.append(marker)
        control.always_visible = True
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.resetToInitalPose)

        self.menu_handler.insert("Reset " + self.name + " to initial configuration", callback=self.resetToInitalPose)
        self.menu_handler.reApply(self.server)
        self.menu_handler.apply(self.server, int_marker.name)