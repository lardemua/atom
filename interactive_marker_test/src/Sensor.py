#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import copy

import numpy as np
import cv2
import tf
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from tf.listener import TransformListener


# ------------------------
#      BASE CLASSES      #
# ------------------------

class TransformationT():
    """ Base class for geometric transformation
    """

    def __init__(self, parent_frame_id, frame_id):
        self.parent_frame_id = parent_frame_id
        self.frame_id = frame_id
        self.matrix = np.identity(4, dtype=np.float)
        self.stamp = rospy.Time.now()

    def __str__(self):
        return 'Transform from ' + str(self.parent_frame_id) + ' to ' + str(self.frame_id) + ' at time ' + str(
            self.stamp) + '\n' + str(self.matrix)

    def getTranslation(self, homogeneous=False):
        if homogeneous:
            return self.matrix[0:4, 3]
        else:
            return self.matrix[0:3, 3]

    def setTranslation(self, value):
        self.matrix[0:3, 3] = value

    def setTranslationFromPosePosition(self, trans):
        self.matrix[0, 3] = trans.x
        self.matrix[1, 3] = trans.y
        self.matrix[2, 3] = trans.z

    def getRotation(self):
        return self.matrix[0:3, 0:3]

    def setRotation(self, matrix):
        self.matrix[0:3, 0:3] = matrix

    def setQuaternion(self, q):
        matrix = copy.deepcopy(tf.transformations.quaternion_matrix(q))
        self.matrix[0:3, 0:3] = matrix[0:3, 0:3]

    def setQuaternionFromPoseQuaternion(self, pose_q):
        q = (pose_q.x, pose_q.y, pose_q.z, pose_q.w)
        matrix = copy.deepcopy(tf.transformations.quaternion_matrix(q))
        self.matrix[0:3, 0:3] = matrix[0:3, 0:3]

    def getQuaternion(self):
        m = copy.deepcopy(self.matrix)
        m[0, 3] = 0
        m[1, 3] = 0
        m[2, 3] = 0
        return tf.transformations.quaternion_from_matrix(m)

    def getEulerAngles(self):
        return tf.transformations.euler_from_matrix(self.matrix)

    def setRodrigues(self, matrix):
        self.setRotation(self.rodriguesToMatrix(matrix))

    def getRodrigues(self):
        return self.matrixToRodrigues(self.getRotation())

    def matrixToRodrigues(self, matrix):
        rods, _ = cv2.Rodrigues(matrix[0:3, 0:3])
        rods = rods.transpose()
        rodrigues = rods[0]
        return rodrigues

    def rodriguesToMatrix(self, r):
        rod = np.array(r, dtype=np.float)
        matrix = cv2.Rodrigues(rod)
        return matrix[0]


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

    def __init__(self, name, server, menu_handler, frame_world, frame_opt_parent, frame_opt_child, frame_sensor):
        print('Creating a new sensor named ' + name)
        self.name = name
        self.server = server
        self.menu_handler = menu_handler
        self.listener = TransformListener()
        self.br = tf.TransformBroadcaster()
        self.timer_callback = rospy.Timer(rospy.Duration(.1), self.publishTFCallback)  # to periodically broadcast
        # transforms
        self.world_link = frame_world
        self.opt_parent_link = frame_opt_parent
        self.opt_child_link = frame_opt_child
        self.sensor_link = frame_sensor
        self.updateAll()  # update all the transformations
        # print('Collected pre, opt and pos transforms.')
        #
        # print('preT:\n' + str(self.preT))
        # print('optT:\n' + str(self.optT))
        # print('posT:\n' + str(self.posT))

        self.optTInitial = copy.deepcopy(self.optT)
        print('\n\n' + ' Acordei' + str(self.optTInitial))
        self.createInteractiveMarker()  # create interactive marker
        print('Created interactive marker.')

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
        self.listener.waitForTransform(parent_link, child_link, stamp, rospy.Duration(1.0))
        (trans, quat) = self.listener.lookupTransform(parent_link, child_link, stamp)
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
        self.marker.scale = 0.2

        self.marker.name = self.name
        self.marker.description = self.name + '_control'

        # insert a box
        control = InteractiveMarkerControl()
        control.always_visible = True

        marker_box = Marker()
        marker_box.type = Marker.SPHERE
        marker_box.scale.x = self.marker.scale * 0.7
        marker_box.scale.y = self.marker.scale * 0.7
        marker_box.scale.z = self.marker.scale * 0.7
        marker_box.color.r = 0
        marker_box.color.g = 1
        marker_box.color.b = 0
        marker_box.color.a = 1.0

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
