#!/usr/bin/env python

# stdlib
import copy

# 3rd-party
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

    def __init__(self, name, server, menu_handler, frame_world, frame_opt_parent, frame_opt_child, frame_sensor,
                 marker_scale):
        print('Creating a new sensor named ' + name)
        self.name = name
        self.server = server
        self.menu_handler = menu_handler
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

        # Add service to make visible / invisible
        # std_srvs / SetBool Service

        self.service_set_visible = rospy.Service('~' + self.name + '/set_visible', std_srvs.srv.SetBool,
                                                 self.setVisible)
        # marker = server.get("marker_name")
        # marker.controls[0].markers[0].color.r = 150
        # marker.controls[0].markers[0].color.g = 0
        # marker.controls[0].markers[0].color.b = 0
        # server.applyChanges()

        # Start publishing now
        self.timer_callback = rospy.Timer(rospy.Duration(.1), self.publishTFCallback)  # to periodically broadcast

    def setVisible(self, request):

        print('setVisible=' + str(request.data) + ' service requested for sensor ' + colorama.Fore.BLUE +
              self.name + colorama.Style.RESET_ALL)

        interactive_marker = self.server.get(self.name)
        for control in interactive_marker.controls:
            if request.data == 1:
                if 'move' in control.name:
                    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
                elif 'rotate' in control.name:
                    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            else:
                control.interaction_mode = InteractiveMarkerControl.NONE

        self.server.insert(interactive_marker)

        self.server.applyChanges()

        response = std_srvs.srv.SetBoolResponse()
        response.success = 1
        response.message = 'All good.'
        return response

    def resetToInitalPose(self):
        self.optT.matrix = self.optTInitial.matrix

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

        self.menu_handler.reApply(self.server)
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

        self.menu_handler.reApply(self.server)
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
        self.menu_handler.apply(self.server, self.marker.name)
