
# stdlib


# 3rd-party
import rospkg
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

# local packages
from atom_calibration.odometry_inspector.additional_tf import Additional_tf


class InteractiveFirstGuess(object):

    def __init__(self, args, dataset, selection):

        self.args = args  # command line arguments
        self.dataset = dataset
        self.selection = selection

        self.server = None  # interactive markers server
        self.menu = None

    def init(self):

        # Init interaction
        self.server = InteractiveMarkerServer('set_odometry_inspector')
        self.menu = MenuHandler()

        self.createInteractiveMarker(self.dataset['calibration_config']['world_link'])

        # sensors and additional tfs to be calibrated
        self.menu.insert("Reset to initial values",
                         callback=self.onResetAll)

        self.menu.reApply(self.server)

        # Generate an interactive marker.
        params = {"frame_world": self.dataset['calibration_config']['world_link'],
                  "frame_opt_parent": self.args['odometry_parent'],
                  "frame_opt_child":  self.args['odometry_child'],
                  "frame_additional_tf": self.args['odometry_child'],
                  "additional_tf_color": (0, 0, 1, 1),
                  "marker_scale": self.args['marker_scale'],
                  "selection": self.selection,
                  "dataset": self.dataset}

        # Append to the list of additional_tfs
        self.additional_tf = Additional_tf('tf', self.server, self.menu, **params)
        print('... done')

        self.server.applyChanges()

    def onResetAll(self, feedback):

        return
        if self.checkAdditionalTfs():
            for additional_tfs in self.additional_tfs:
                additional_tfs.resetToInitalPose()

    def createInteractiveMarker(self, world_link):
        marker = InteractiveMarker()
        marker.header.frame_id = world_link
        trans = (1, 0, 1)
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        quat = (0, 0, 0, 1)
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

        marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_3D

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
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
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

        # self.server.insert(marker, self.onSaveInitialEstimate)

        self.menu.apply(self.server, marker.name)

    def checkAdditionalTfs(self):
        return self.config['additional_tfs'] is not None
