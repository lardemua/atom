#!/usr/bin/env python
import argparse
import collections

import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.listener import TransformListener
from urdf_parser_py.urdf import URDF
import rospkg

server = None
menu_handler = MenuHandler()
br = tf.TransformBroadcaster()
marker_poses = []
robot = []
optimization_parent_link = ""


class MarkerPoseC:
    def __init__(self, position, orientation, frame_id, child_frame_id):
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

    def __str__(self):
        return str(self.position) + "\n" + str(self.orientation)

    __repr__ = __str__


class Transform:
    def __init__(self, translation, rotation, parent_link, child_link):
        self.translation = translation
        self.rotation = rotation
        self.parent_link = parent_link
        self.child_link = child_link

    def __str__(self):
        return "tf from " + str(self.parent_link) + " to " + str(self.child_link)
    __repr__ = __str__


class SensorTFs:
    def __init__(self, sensor_name, pre_tfs, pos_tfs):
        self.sensor_name = sensor_name
        self.pre_tfs = pre_tfs
        self.pos_tfs = pos_tfs

    def __str__(self):
        return "\n\n" + str(self.sensor_name) + ":\npre_tfs: " + str(self.pre_tfs) + "\npos_tfs: " + str(self.pos_tfs)

    __repr__ = __str__


class InitialGuess:
    def __init__(self, sensor_name, position, orientation):
        self.sensor_name = sensor_name
        self.position = position
        self.orientation = orientation

    def __str__(self):
        return "\n\n" + str(self.sensor_name) + ":\nposition: " + str(self.position) + "\norientation: " + str(self.orientation)

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
    global marker_poses

    for mp in marker_poses:
        if feedback.marker_name == mp.child_frame_id:
            mp.position = feedback.pose.position
            mp.orientation = feedback.pose.orientation
            break
    menu_handler.reApply(server)
    server.applyChanges()


def menuFeedback(feedback):
    global handle, robot, optimization_parent_link
    handle = feedback.menu_entry_id
    listener2 = TransformListener()
    rospy.sleep(1)
    if handle == 1:
        all_initial_guess = []
        for joint in robot.joints:
            for sensor in robot.sensors:
                if sensor.parent == joint.child:
                    optimization_parent_link = joint.parent
            for mp in marker_poses:
                (trans, rot) = listener2.lookupTransform(optimization_parent_link, mp.child_frame_id, rospy.Time(0))
                if joint.child + "_first_guess" == mp.child_frame_id:
                    joint.origin.xyz[0] = trans[0]
                    joint.origin.xyz[1] = trans[1]
                    joint.origin.xyz[2] = trans[2]
                    joint.origin.rpy[0] = rot[0]
                    joint.origin.rpy[1] = rot[1]
                    joint.origin.rpy[2] = rot[2]
                    ig = InitialGuess(joint.child, Point(float(trans[0]), float(trans[1]), float(trans[2])), Quaternion(
                        float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])))
                    all_initial_guess.append(ig)

    xml_string = robot.to_xml_string()

    f = open(rospack.get_path('interactive_marker_test') + "/urdf/atlas2_macro_first_guess.urdf.xacro", "w")
    f.write(xml_string)
    f.close()
    print(all_initial_guess)


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
    global optimization_parent_link

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = optimization_parent_link
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
    menu_handler.insert("Save sensors configuration", callback=menuFeedback)


if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--common_frame", help='Name of the reference frame wich is common to all sensors. Usually '
                                                 'it is the world or base_link.', type=str, required=True)
    args = vars(ap.parse_args())

    # print(args['common_frame'])

    rospy.init_node("sensors_first_guess")
    listener = TransformListener()

    rate = rospy.Rate(10.0)  # 10 Hz

    robot_description = rospy.get_param('/robot_description')

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    robot = URDF.from_parameter_server()

    server = InteractiveMarkerServer("sensors_first_guess")

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.1), publishTFsCallback)

    rospy.sleep(0.5)

    count = 0
    all_sensors_tfs = []
    # parsing of robot description
    for sensor in robot.sensors:

        print('\n\nSensor name is ' + sensor.name)

        print('Sensor data reference frame is ' + sensor.parent)

        print('Optimization child is ' + sensor.parent)

        for joint in robot.joints:
            if sensor.parent == joint.child:
                optimization_parent_link = joint.parent

        print('Optimization parent  ' + optimization_parent_link)

        pre_transforms = []
        found_master_link = 0
        for joint2 in robot.joints:
            if joint2.parent == optimization_parent_link and optimization_parent_link != args['common_frame']:
                last_joint = joint2
                while found_master_link == 0:
                    for joint3 in robot.joints:
                        if joint3.child == last_joint.parent:
                            (trans, rot) = listener.lookupTransform(joint3.parent, joint3.child, rospy.Time(0))
                            pre_tf = Transform(trans, rot, joint3.parent, joint3.child)
                            pre_transforms.append(pre_tf)
                            last_joint = joint3
                            if joint3.parent == args['common_frame']:
                                found_master_link = 1
                            break
                break
        # print(pre_transforms)

        pos_transforms = []
        found_final_link = 0
        for joint2 in robot.joints:
            if joint2.child == sensor.parent:  # and joint2.child != sensor.parent:
                last_joint = joint2
                while found_final_link == 0:
                    for joint3 in robot.joints:
                        if joint3.parent == last_joint.child:
                            (trans, rot) = listener.lookupTransform(joint3.parent, joint3.child, rospy.Time(0))
                            pos_tf = Transform(trans, rot, joint3.parent, joint3.child)
                            pos_transforms.append(pos_tf)
                            last_joint = joint3
                            if joint3.child == sensor.name:
                                found_final_link = 1
                            break
                    break
                break
        # print(pos_transforms)
        sensor_tfs = SensorTFs(sensor.name, pre_transforms, pos_transforms)
        all_sensors_tfs.append(sensor_tfs)

        while not rospy.is_shutdown():
            (trans, rot) = listener.lookupTransform(optimization_parent_link, str(sensor.parent), rospy.Time(0))
            orientation = Quaternion(float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3]))
            position = Point(float(trans[0]), float(trans[1]), float(trans[2]))
            make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, position, orientation, sensor.name + "_first_guess",
                           1)
            mp = MarkerPoseC(position, orientation, optimization_parent_link, sensor.name + "_first_guess")
            marker_poses.append(mp)
            break

        count = count + 1

    print('\n\nNumber of sensors: ' + str(count))
    print("\n" + str(all_sensors_tfs))

    initMenu()

    server.applyChanges()

    rospy.spin()
