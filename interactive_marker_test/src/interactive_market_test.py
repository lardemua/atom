#!/usr/bin/env python
import collections

import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener

server = None
menu_handler = MenuHandler()
# h_first_entry = 0
br = tf.TransformBroadcaster()
marker_poses = []

# MarkerPoseT = collections.namedtuple('MarkerPoseT', 'position orientation frame_id child_frame_id')


class MarkerPoseC:
    def __init__(self, position, orientation, frame_id, child_frame_id):
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

    def __str__(self):
        return str(self.position) + "\n" + str(self.orientation)

    __repr__ = __str__


def publishTFsCallback(msg):
    global br, marker_poses

    for mp in marker_poses:
        br.sendTransform((mp.position.x, mp.position.y, mp.position.z),
                         (mp.orientation.x, mp.orientation.y, mp.orientation.z,
                          mp.orientation.w),
                         rospy.Time.now(), mp.child_frame_id, mp.frame_id)

    # rospy.Time.now()


def processFeedback(feedback):
    # print("feedback" + str(feedback))

    global marker_poses
    # print(marker_poses)

    for mp in marker_poses:
        if feedback.marker_name == mp.child_frame_id:
            mp.position = feedback.pose.position
            mp.orientation = feedback.pose.orientation
            break

    menu_handler.reApply(server)

    server.applyChanges()

def menuFeedback(feedback):
    global handle, marker_poses
    handle = feedback.menu_entry_id
    listener2 = TransformListener()
    rospy.sleep(1)
    if handle == 1:
        for mp in marker_poses:
            (trans, rot) = listener2.lookupTransform('base_link', mp.child_frame_id, rospy.Time(0))
            print(mp.child_frame_id + ":")
            print("   Position: " + str(trans))
            print("   Orientation: (" + str(rot))

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


def make6DofMarker(interaction_mode, position, orientation, name, show_6dof=0):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.pose.orientation = orientation
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
    menu_handler.insert("Get updated markers pose", callback=menuFeedback)


if __name__ == "__main__":

    rospy.init_node("basic_controls_with_menu")
    # br = TransformBroadcaster()
    listener = TransformListener()

    rate = rospy.Rate(10.0)  # 10 Hz

    robot_description = rospy.get_param('/robot_description')

    from urdf_parser_py.urdf import URDF

    robot = URDF.from_parameter_server()

    server = InteractiveMarkerServer("basic_controls_with_menu")

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.1), publishTFsCallback)

    count = 0

    # parsing of robot description
    for sensor in robot.sensors:
        while not rospy.is_shutdown():
            (trans, rot) = listener.lookupTransform('base_link', str(sensor.name), rospy.Time(0))
            orientation = Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
            position = Point(float(trans[0]), float(trans[1]), float(trans[2]))
            make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, position, orientation, "Marker" + str(count + 1), 1)
            mp = MarkerPoseC(position, orientation, 'base_link', 'Marker' + str(count + 1))
            marker_poses.append(mp)
            break

        count = count + 1

    print('Number of sensors: ' + str(count))

    initMenu()

    server.applyChanges()

    rospy.spin()
