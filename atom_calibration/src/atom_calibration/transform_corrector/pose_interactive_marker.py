#!/usr/bin/env python

# stdlib
import copy

# 3rd-party
from atom_core.naming import generateKey
from atom_core.utilities import atomError
from colorama import Fore, Style
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import *

# from transformation_t import TransformationT
from atom_calibration.initial_estimate.transformation_t import TransformationT

# ------------------------
#      BASE CLASSES      #
# ------------------------


class PoseInteractiveMarker:

    def __init__(self, name, dataset, selection, server,
                 world_link, parent, child, is_fixed,
                 marker_scale=0.8, two_dimensional_mode=False):
        """_summary_

        Args:
            name (_type_): _description_
            dataset (_type_): _description_
            selection (_type_): _description_
            server (_type_): _description_
            world_link (_type_): _description_
            parent (_type_): _description_
            child (_type_): _description_
            is_fixed (bool): If transform is fixed the changes from moving the interactive marker will affect all the collections in the dataset.
            marker_scale (float, optional): _description_. Defaults to 0.8.
            two_dimensional_mode (bool, optional): _description_. Defaults to False.
        """

        self.name = name
        self.visible = True
        self.server = server
        self.selection = selection
        self.dataset = dataset
        self.marker_scale = marker_scale
        self.two_dimensional_mode = two_dimensional_mode
        self.is_fixed = is_fixed

        # transforms
        self.world_link = world_link
        self.opt_parent_link = parent
        self.opt_child_link = child

        self.transformFromDataset()  # update all the transformations
        # print('optT:\n' + str(self.optT))

        self.createInteractiveMarker()  # create interactive marker
        print('Created interactive marker ' + Fore.BLUE + self.name + Style.RESET_ALL)

    def markerFeedback(self, feedback):
        # print(' marker  ' + self.name + ' received feedback')

        # Update transformation from marker pose
        self.optT.setTranslationFromPosePosition(feedback.pose.position)
        self.optT.setQuaternionFromPoseQuaternion(feedback.pose.orientation)

        # Update transformation in dataset
        self.transformToDataset()

    def transformFromDataset(self):

        transform_key = generateKey(parent=self.opt_parent_link, child=self.opt_child_link)
        quat = self.dataset['collections'][
            self.selection['collection_key']]['transforms'][transform_key]['quat']
        trans = self.dataset['collections'][
            self.selection['collection_key']]['transforms'][transform_key]['trans']

        T = TransformationT(self.opt_parent_link, self.opt_child_link)
        T.setTranslation(trans)
        T.setQuaternion(quat)

        self.optT = T

    def updateMarkersFromDataset(self):

        # Read transformation from dataset
        self.transformFromDataset()

        # Get transformation
        trans = self.optT.getTranslation()
        quat = self.optT.getQuaternion()

        # Set marker pose
        marker = self.server.get(self.name)
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        self.server.insert(marker)
        self.server.applyChanges()

    def transformToDataset(self):

        # Write new values to the dataset
        quat = list(self.optT.getQuaternion())
        trans = list(self.optT.getTranslation())
        transform_key = generateKey(parent=self.opt_parent_link, child=self.opt_child_link)

        if self.is_fixed:  # if fixed all collections must be changed
            for collection_key in self.dataset['collections']:
                self.dataset['collections'][collection_key]['transforms'][transform_key]['quat'] = quat
                self.dataset['collections'][collection_key]['transforms'][transform_key][
                    'trans'] = trans
        else:
            self.dataset['collections'][self.selection['collection_key']
                                        ]['transforms'][transform_key]['quat'] = quat
            self.dataset['collections'][self.selection['collection_key']
                                        ]['transforms'][transform_key]['trans'] = trans

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
        marker_box.scale.x = self.marker.scale * 0.03
        marker_box.scale.y = self.marker.scale * 0.03
        marker_box.scale.z = self.marker.scale * 0.03
        marker_box.color.r = 0
        marker_box.color.g = 1
        marker_box.color.b = 0
        marker_box.color.a = 1.0

        control.markers.append(marker_box)
        self.marker.controls.append(control)

        self.marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE

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
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
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

        if not self.two_dimensional_mode:
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
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation_mode = InteractiveMarkerControl.FIXED
            self.marker.controls.append(control)

        self.server.insert(self.marker, self.markerFeedback)
        self.server.applyChanges()
