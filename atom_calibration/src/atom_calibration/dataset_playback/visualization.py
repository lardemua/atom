"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

import copy
import math
import struct
from re import I
from tokenize import generate_tokens

# 3rd-party
import cv2
import cv_bridge
import numpy as np
import atom_core.ros_numpy

# import numpy as np  # TODO Eurico, line  fails if I don't do this
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
from atom_calibration.calibration.objective_function import *
from atom_calibration.collect.label_messages import *
from atom_calibration.dataset_playback.depth_manual_labeling import drawLabelsOnImage, normalizeDepthImage
from atom_core.cache import Cache
from atom_core.config_io import execute, uriReader
from atom_core.xacro_io import readXacroFile
from atom_core.dataset_io import (genCollectionPrefix, getCvImageFromDictionary, getCvImageFromDictionaryDepth,
                                  getPointCloudMessageFromDictionary)
from atom_core.drawing import drawCross2D, drawSquare2D
from atom_core.naming import generateLabeledTopic, generateName
from atom_core.rospy_urdf_to_rviz_converter import urdfToMarkerArray
from colorama import Fore, Style
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from matplotlib import cm
from rospy_message_converter import message_converter
from sensor_msgs.msg import PointCloud2, PointField, sensor_msgs
from std_msgs.msg import ColorRGBA, Header, UInt8MultiArray
from urdf_parser_py.urdf import URDF

# stdlib
from visualization_msgs.msg import Marker, MarkerArray

# own packages

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def createDepthMarkers3D(dataset, sensor_key, collection_key, color=(255, 255, 0, 0.5),
                         stamp=None, cached=False, frame_id=None, namespace=None):

    if stamp is None:
        stamp = rospy.Time.now()

    collection = dataset['collections'][collection_key]

    if frame_id is None:
        frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])

    if namespace is None:
        namespace = str(collection_key) + '-' + str(sensor_key)

    markers = MarkerArray()
    # Add labelled points to the marker
    marker = Marker(
        header=Header(frame_id=frame_id, stamp=stamp),
        ns=namespace,
        id=0, frame_locked=True, type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
        pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
        scale=Vector3(x=0.01, y=0.01, z=0.01),
        color=ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3]))

    if cached:
        points = getPointsInDepthSensorAsNPArray(collection_key, sensor_key, 'idxs', dataset)
    else:
        points = getPointsInDepthSensorAsNPArrayNonCached(collection_key, sensor_key, 'idxs', dataset)

    for idx in range(0, points.shape[1]):
        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

    markers.markers.append(copy.deepcopy(marker))

    # Add limit points to the marker, this time with larger cubes
    marker = Marker(header=Header(frame_id=frame_id, stamp=stamp),
                    ns=namespace + '-limit_points', id=0,
                    frame_locked=True,
                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.05, y=0.05, z=0.05),
                    color=ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3]))

    if cached:
        points = getPointsInDepthSensorAsNPArray(collection_key, sensor_key, 'idxs_limit_points', dataset)
    else:
        points = getPointsInDepthSensorAsNPArrayNonCached(collection_key, sensor_key, 'idxs_limit_points', dataset)

    for idx in range(0, points.shape[1]):
        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

    markers.markers.append(copy.deepcopy(marker))

    return markers


def getPointsInSensorAsNPArray_local(_collection_key, _sensor_key, _label_key, _dataset):
    # TODO: #395 Daniel, we should you told me about this one but I would like to talk to you again ... although this function is somewhere else, in the other place it uses the dataset as cache...
    cloud_msg = getPointCloudMessageFromDictionary(
        _dataset['collections'][_collection_key]['data'][_sensor_key])
    idxs = _dataset['collections'][_collection_key]['labels'][_sensor_key][_label_key]
    pc = atom_core.ros_numpy.numpify(cloud_msg)[idxs]
    points = np.zeros((4, pc.shape[0]))
    points[0, :] = pc['x']
    points[1, :] = pc['y']
    points[2, :] = pc['z']
    points[3, :] = 1
    return points


def getCvImageFromCollectionSensor(collection_key, sensor_key, dataset):
    dictionary = dataset['collections'][collection_key]['data'][sensor_key]
    return getCvImageFromDictionary(dictionary)


def getCvDepthImageFromCollectionSensor(collection_key, sensor_key, dataset, scale=1000.0):
    dictionary = dataset['collections'][collection_key]['data'][sensor_key]
    return getCvImageFromDictionaryDepth(dictionary, scale=scale)


def createPatternMarkers(frame_id, ns, collection_key, now, dataset, graphics):
    markers = MarkerArray()

    # Draw pattern frame lines_sampled (top, left, right, bottom)
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-frame_sampled', id=0, frame_locked=True,
                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(
                        0),
                    pose=Pose(position=Point(x=0, y=0, z=0),
                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.01, y=0.01, z=0.01),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0))

    pts = []
    pts.extend(dataset['patterns']['frame']['lines_sampled']['left'])
    pts.extend(dataset['patterns']['frame']['lines_sampled']['right'])
    pts.extend(dataset['patterns']['frame']['lines_sampled']['top'])
    pts.extend(dataset['patterns']['frame']['lines_sampled']['bottom'])
    for pt in pts:
        marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))

    markers.markers.append(marker)

    # Draw corners
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-corners', id=0, frame_locked=True,
                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(
                        0),
                    pose=Pose(position=Point(x=0, y=0, z=0),
                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.02, y=0.02, z=0.02),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0))

    for idx_corner, pt in enumerate(dataset['patterns']['corners']):
        marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))
        marker.colors.append(ColorRGBA(r=graphics['pattern']['colormap'][idx_corner, 0],
                                       g=graphics['pattern']['colormap'][idx_corner, 1],
                                       b=graphics['pattern']['colormap'][idx_corner, 2], a=1))

    markers.markers.append(marker)

    # Draw transitions
    # TODO we don't use this anymore, should we draw it? Perhaps it will be used for 2D Lidar ...
    # marker = Marker(header=Header(frame_id=frame_id, stamp=now),
    #                 ns=ns + '-transitions', id=0, frame_locked=True,
    #                 type=Marker.POINTS, action=Marker.ADD, lifetime=rospy.Duration(0),
    #                 pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
    #                 scale=Vector3(x=0.015, y=0.015, z=0),
    #                 color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
    #                                 g=graphics['collections'][collection_key]['color'][1],
    #                                 b=graphics['collections'][collection_key]['color'][2], a=1.0))
    #
    # pts = dataset['patterns']['transitions']['vertical']
    # pts.extend(dataset['patterns']['transitions']['horizontal'])
    # for pt in pts:
    #     marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))
    #
    # markers.markers.append(marker)

    # Draw the mesh, if one is provided
    if not dataset['calibration_config']['calibration_pattern']['mesh_file'] == "":
        # rgba = graphics['collections'][collection_key]['color']
        # color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=1))

        # print('Got the mesh it is: ' + dataset['calibration_config']['calibration_pattern']['mesh_file'])
        m = Marker(header=Header(frame_id=frame_id, stamp=now),
                   ns=str(collection_key) + '-mesh', id=0, frame_locked=True,
                   type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(
                       0),
                   pose=Pose(position=Point(x=0, y=0, z=0),
                             orientation=Quaternion(x=0, y=0, z=0, w=1)),
                   scale=Vector3(x=1.0, y=1.0, z=1.0),
                   color=ColorRGBA(r=1, g=1, b=1, a=1))

        mesh_file, _, _ = uriReader(
            dataset['calibration_config']['calibration_pattern']['mesh_file'])
        m.mesh_resource = 'file://' + mesh_file  # mesh_resource needs uri format
        m.mesh_use_embedded_materials = True
        markers.markers.append(m)

    return markers  # return markers


def setupVisualization(dataset, args, selected_collection_key):
    """
    Creates the necessary variables in a dictionary "dataset_graphics", which will be passed onto the visualization
    function
    """

    # Create a python dictionary that will contain all the visualization related information
    graphics = {'collections': {}, 'pattern': {}, 'ros': {'sensors': {}}, 'args': args}

    # Parse xacro description file
    description_file, _, _ = uriReader(
        dataset['calibration_config']['description_file'])
    rospy.loginfo('Reading description file ' + description_file + '...')
    # TODO not sure this should be done because of the use_tfs functionality ...
    xml_robot = readXacroFile(description_file)

    # Initialize ROS stuff
    rospy.init_node("dataset_playback")
    # graphics['ros']['tf_broadcaster'] = tf.TransformBroadcaster()
    graphics['ros']['tf_broadcaster'] = tf2_ros.TransformBroadcaster()

    # Sleep a litle to make sure the time.now() returns a correct time.
    # rospy.sleep(0.2)
    now = rospy.Time.now()

    graphics['ros']['publisher_models'] = rospy.Publisher(
        '~robot_meshes', MarkerArray, queue_size=0, latch=True)
    # Analyse xacro and figure out which transforms are static (always the same over the optimization), and which are
    # not. For fixed we will use a static transform publisher.
    # for collection_key, collection in dataset['collections'].items():
    #     for transform_key, transform in collection['transforms'].items():
    #         parent = transform['parent']
    #         child = transform['child']
    #
    #
    #         for sensor_key, sensor in dataset['sensors'].items(): # is the transformation being optimized?
    #             if joint.parent == sensor['calibration_parent'] and joint.child == sensor['calibration_child']:
    #
    #
    #
    #         for joint in xml_robot.joints:
    #             if joint.parent == parent and joint.child == child:
    #                 print(transform)
    #                 print(joint)
    #
    #
    #
    #                 if joint.type == 'fixed':
    #                     transform['fixed'] = True
    #                 else:
    #                     transform['fixed'] = False

    # Create colormaps to be used for coloring the elements. Each collection contains a color, each sensor likewise.
    pattern = dataset['calibration_config']['calibration_pattern']
    graphics['pattern']['colormap'] = cm.gist_rainbow(
        np.linspace(0, 1, pattern['dimension']['x'] * pattern['dimension']['y']))

    # graphics['collections']['colormap'] = cm.tab20b(np.linspace(0, 1, len(dataset['collections'].keys())))
    graphics['collections']['colormap'] = cm.Pastel2(
        np.linspace(0, 1, len(dataset['collections'].keys())))
    for idx, collection_key in enumerate(sorted(dataset['collections'].keys())):
        graphics['collections'][str(collection_key)] = {
            'color': graphics['collections']['colormap'][idx, :]}

    # color_map_sensors = cm.gist_rainbow(np.linspace(0, 1, len(dataset['sensors'].keys())))
    # for idx, sensor_key in enumerate(sorted(dataset['sensors'].keys())):
    #     dataset['sensors'][str(sensor_key)]['color'] = color_map_sensors[idx, :]

    # ----------------------------------------------------------------------------------------
    # Create 2D Labels  (only for rgb and depth)
    # Note: Republish a new image at every visualization, because the labels must redrawn as they change position
    # ----------------------------------------------------------------------------------------
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['modality'] in ['rgb', 'depth']:
            graphics['collections'][sensor_key] = {}

            # Image msg setup.
            labeled_topic = generateLabeledTopic(dataset['sensors'][sensor_key]['topic'], type='2d')
            graphics['collections'][sensor_key]['publisher'] = rospy.Publisher(
                labeled_topic, sensor_msgs.msg.Image, queue_size=0, latch=True)

            # Camera info msg setup.
            labeled_topic = generateLabeledTopic(
                dataset['sensors'][sensor_key]['topic'], type='2d') + '/camera_info'
            graphics['collections'][sensor_key]['publisher_camera_info'] = rospy.Publisher(
                labeled_topic, sensor_msgs.msg.CameraInfo, queue_size=0, latch=True)

    # ----------------------------------------------------------------------------------------
    # Create 3D Labels  (only for lidar3d and depth)
    # Note: Republish new labeled point clouds because the labels must be redrawn as they change position
    # ----------------------------------------------------------------------------------------

    graphics['ros']['PubPointCloud'] = dict()
    for sensor_key, sensor in dataset['sensors'].items():
        # print('Sensor ' + sensor_key)

        if sensor['modality'] not in ['lidar2d', 'lidar3d', 'depth']:
            continue

        markers = MarkerArray()
        graphics['ros']['sensors'][sensor_key] = {'collections': {}}

        for collection_key, collection in dataset['collections'].items():
            # print('Collection ' + collection_key)

            # Removed because of https://github.com/lardemua/atom/issues/515
            # if not collection['labels'][str(sensor_key)]['detected']:  # not detected by sensor in collection
                # continue

            graphics['ros']['sensors'][sensor_key]['collections'][collection_key] = {}

            # when the sensor has no label, the 'idxs_limit_points' does not exist!!!
            # TODO this is not correct I think ...
            if 'idxs_limit_points' not in collection['labels'][str(sensor_key)]:
                collection['labels'][str(sensor_key)]['idxs_limit_points'] = []

            if collection['labels'][str(sensor_key)]['idxs'] != []:
                collection['labels'][str(sensor_key)]['detected'] = True

            # if sensor['msg_type'] == 'LaserScan':  # -------- Publish the laser scan data ------------------------------
            if sensor['modality'] == 'lidar2d':
                frame_id = genCollectionPrefix(
                    collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.POINTS, action=Marker.ADD, lifetime=rospy.Duration(
                    0),
                    pose=Pose(position=Point(
                        x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.03, y=0.03, z=0),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0)
                )

                # Get laser points that belong to the chessboard (labelled)
                idxs = collection['labels'][sensor_key]['idxs']
                rhos = [collection['data'][sensor_key]['ranges'][idx]
                        for idx in idxs]
                thetas = [collection['data'][sensor_key]['angle_min'] +
                          collection['data'][sensor_key]['angle_increment'] * idx for idx in idxs]

                for idx, (rho, theta) in enumerate(zip(rhos, thetas)):
                    marker.points.append(
                        Point(x=rho * math.cos(theta), y=rho * math.sin(theta), z=0))

                markers.markers.append(copy.deepcopy(marker))

                # Draw extrema points
                marker.ns = str(collection_key) + '-' + str(sensor_key)
                marker.type = Marker.SPHERE_LIST
                marker.id = 1
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 0.5
                marker.points = [marker.points[0], marker.points[-1]]

                markers.markers.append(copy.deepcopy(marker))

                # Draw detected edges
                marker.ns = str(collection_key) + '-' + str(sensor_key)
                marker.type = Marker.CUBE_LIST
                marker.id = 2
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 0.5

                marker.points = []  # Reset the list of marker points
                # add edge points
                for edge_idx in collection['labels'][sensor_key]['edge_idxs']:
                    p = Point()
                    p.x = rhos[edge_idx] * math.cos(thetas[edge_idx])
                    p.y = rhos[edge_idx] * math.sin(thetas[edge_idx])
                    p.z = 0
                    marker.points.append(p)
                markers.markers.append(copy.deepcopy(marker))

            # if sensor['msg_type'] == 'PointCloud2':  # -------- Publish the velodyne data ------------------------------
            elif sensor['modality'] == 'lidar3d':

                # print('Entering modality 3d for sensor ' + sensor_key + ' collection ' + collection_key)
                # Add labelled points to the marker
                frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])

                # Add 3D lidar data
                original_pointcloud_msg = getPointCloudMessageFromDictionary(
                    dataset['collections'][collection_key]['data'][sensor_key])

                # print('original_pointcloud_msg.width = ' + str(original_pointcloud_msg.width))
                # print('original_pointcloud_msg.height = ' + str(original_pointcloud_msg.height))

                # exit(0)
                final_pointcloud_msg = PointCloud2(header=Header(frame_id=frame_id, stamp=now),
                                                   height=original_pointcloud_msg.height,
                                                   width=original_pointcloud_msg.width,
                                                   fields=original_pointcloud_msg.fields,
                                                   is_bigendian=original_pointcloud_msg.is_bigendian,
                                                   point_step=original_pointcloud_msg.point_step,
                                                   row_step=original_pointcloud_msg.row_step,
                                                   data=original_pointcloud_msg.data,
                                                   is_dense=original_pointcloud_msg.is_dense)

                labeled_topic = generateLabeledTopic(dataset['sensors'][sensor_key]['topic'], type='3d')
                graphics['ros']['sensors'][sensor_key]['PubPointCloud'] = rospy.Publisher(
                    labeled_topic, PointCloud2, queue_size=0, latch=True)
                graphics['ros']['sensors'][sensor_key]['collections'][collection_key]['PointClouds'] = final_pointcloud_msg

    # 3D labels for depth sensors ------------------------------------------------------------------------
    # sensor_key + '-Labels-3D'
    print('Creating 3D labels for depth sensors ...')
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['modality'] not in ['depth']:  # only depth sensors
            continue


        # if not collection['labels'][sensor_key]['detected']:  # not detected by sensor in collection
        #     continue

        print(sensor_key)

        # Create a single publisher for the sensor (will use this one for all collections)
        labeled_topic = generateLabeledTopic(dataset['sensors'][sensor_key]['topic'], type='3d')
        graphics['ros']['sensors'][sensor_key]['PubDepthMarkers'] = rospy.Publisher(
            labeled_topic, MarkerArray, queue_size=0, latch=True)

    # -----------------------------------------------------------------------------------------------------
    # -------- Robot meshes
    # -----------------------------------------------------------------------------------------------------
    graphics['ros']['robot_mesh_markers'] = {'collections': {}}
    for collection_key, collection in dataset['collections'].items():
        rgba = [.5, .5, .5, 1]  # best color we could find
        markers = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                    namespace='robot_mesh', rgba=rgba)
        graphics['ros']['robot_mesh_markers']['collections'][collection_key] = markers

    # Create LaserBeams Publisher -----------------------------------------------------------
    # This one is recomputed every time in the objective function, so just create the generic properties.
    markers = MarkerArray()

    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            # chess not detected by sensor in collection
            if not collection['labels'][sensor_key]['detected']:
                continue
            # if sensor['msg_type'] == 'LaserScan' or sensor['msg_type'] == 'PointCloud2':
            if sensor['modality'] == 'lidar2d' or sensor['modality'] == 'lidar3d':
                frame_id = genCollectionPrefix(
                    collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.LINE_LIST, action=Marker.ADD, lifetime=rospy.Duration(
                    0),
                    pose=Pose(position=Point(
                        x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.001, y=0, z=0),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0)
                )
                markers.markers.append(marker)

    graphics['ros']['MarkersLaserBeams'] = markers
    graphics['ros']['PubLaserBeams'] = rospy.Publisher(
        '~LaserBeams', MarkerArray, queue_size=0, latch=True)

    # Create Miscellaneous MarkerArray -----------------------------------------------------------
    markers = MarkerArray()

    # Text signaling the anchored sensor
    for _sensor_key, sensor in dataset['sensors'].items():
        if _sensor_key == dataset['calibration_config']['anchored_sensor']:
            marker = Marker(header=Header(frame_id=str(_sensor_key), stamp=now),
                            ns=str(_sensor_key), id=0, frame_locked=True,
                            type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, lifetime=rospy.Duration(
                0),
                pose=Pose(position=Point(x=0, y=0, z=0.2),
                          orientation=Quaternion(x=0, y=0, z=0, w=1)),
                scale=Vector3(x=0.0, y=0.0, z=0.1),
                color=ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0), text='Anchored')

            markers.markers.append(marker)

    graphics['ros']['MarkersMiscellaneous'] = markers
    graphics['ros']['PubMiscellaneous'] = rospy.Publisher(
        '~Miscellaneous', MarkerArray, queue_size=0, latch=True)
    # Publish only once in latched mode
    graphics['ros']['PubMiscellaneous'].publish(
        graphics['ros']['MarkersMiscellaneous'])
    graphics['ros']['Rate'] = rospy.Rate(10)
    graphics['ros']['Rate'] = rospy.Rate(10)
    # tfs need to be published at high frequencies. On the other hand, robot markers
    graphics['ros']['Counter'] = 0
    # should be published at low frequencies. This counter will serve to control this mechanism.

    return graphics


def visualizationFunction(models, selection, clicked_points=None):
    # print(Fore.RED + 'Visualization function called.' + Style.RESET_ALL)
    # Get the data from the meshes
    dataset = models['dataset']
    args = models['args']
    collections = models['dataset']['collections']
    sensors = models['dataset']['sensors']
    config = models['dataset']['calibration_config']
    graphics = models['graphics']

    selected_collection_key = selection['collection_key']
    previous_selected_collection_key = selection['previous_collection_key']

    collection = dataset['collections'][selected_collection_key]

    # print("args['initial_pose_ghost'])" + str(args['initial_pose_ghost']))

    now = rospy.Time.now()  # time used to publish all visualization messages

    transfoms = []
    for collection_key, collection in collections.items():

        # To have a fully connected tree, must connect the instances of the tf tree of every collection into a single
        # tree. We do this by publishing an identity transform between the configured world link and hte world link
        # of each collection.
        parent = config['world_link']
        child = generateName(config['world_link'], prefix='c' + collection_key)

        transform = TransformStamped(header=Header(frame_id=parent, stamp=now),
                                     child_frame_id=child,
                                     transform=Transform(translation=Vector3(x=0, y=0, z=0),
                                                         rotation=Quaternion(x=0, y=0, z=0, w=1)))
        transfoms.append(transform)

        for transform_key, transform in collection['transforms'].items():
            parent = generateName(
                transform['parent'], prefix='c' + collection_key)
            child = generateName(
                transform['child'], prefix='c' + collection_key)
            x, y, z = transform['trans']
            qx, qy, qz, qw = transform['quat']
            transform = TransformStamped(header=Header(frame_id=parent, stamp=now),
                                         child_frame_id=child,
                                         transform=Transform(translation=Vector3(x=x, y=y, z=z),
                                                             rotation=Quaternion(x=qx, y=qy, z=qz, w=qw)))
            transfoms.append(transform)

    graphics['ros']['tf_broadcaster'].sendTransform(transfoms)

    # print("graphics['ros']['Counter'] = " + str(graphics['ros']['Counter']))
    if graphics['ros']['Counter'] < 5:
        graphics['ros']['Counter'] += 1
        return None
    else:
        graphics['ros']['Counter'] = 0

    # Update markers stamp, so that rviz uses newer transforms to compute their poses.
    # for marker in graphics['ros']['robot_mesh_markers'].markers:
        # marker.header.stamp = now

    # Publish the meshes
    markers = graphics['ros']['robot_mesh_markers']['collections'][selected_collection_key]
    for marker in markers.markers:
        marker.header.stamp = now

    graphics['ros']['publisher_models'].publish(markers)

    # Update timestamp for labeled markers
    #     for sensor_key in graphics['ros']['sensors']:
    #         if not dataset['sensors'][sensor_key]['modality'] in ['lidar3d']:  # TODO add depth and lidar2d
    #             continue
    #
    #         for marker in graphics['ros']['sensors'][sensor_key]['collections'][selected_collection_key][
    #                 'MarkersLabeled'].markers:
    #             marker.header.stamp = now
    #
    # Update timestamp for laser beams markers
    for marker in graphics['ros']['MarkersLaserBeams'].markers:
        marker.header.stamp = now

    # Publish 3d markers for depth sensors -------------------------------------------------------------------
    for sensor_key in graphics['ros']['sensors']:
        if not dataset['sensors'][sensor_key]['modality'] in ['depth']:  # only for depth sensors
            continue
        # print("Creating markers")
        # print(sensor_key)

        # if not dataset['collections'][selected_collection_key]['labels'][sensor_key]['detected']:
        #     continue
        #
        # if not collection['labels'][sensor_key]['detected']:  # not detected by sensor in collection
        #     continue

        color = (graphics['collections'][collection_key]['color'][0], graphics['collections']
                 [collection_key]['color'][1], graphics['collections'][collection_key]['color'][2], 0.5)
        markers = MarkerArray()
        markers = createDepthMarkers3D(dataset, sensor_key, selected_collection_key, color=color,
                                       stamp=None, cached=False, frame_id=None, namespace=sensor_key)

        graphics['ros']['sensors'][sensor_key]['PubDepthMarkers'].publish(markers)

    # Update timestamp for pointcloud2 message
    for sensor_key in graphics['ros']['sensors']:
        if not dataset['sensors'][sensor_key]['modality'] in ['lidar3d']:  # TODO add  lidar2d
            continue




        # Create a new point cloud to publish which has the new label idxs in green, idxs_limits in dark green.
        # Also, change intensity channel according to the idxs of the sensor
        pointcloud_msg = graphics['ros']['sensors'][sensor_key]['collections'][selected_collection_key]['PointClouds']
        pointcloud_msg.header.stamp = now  # update time stamp.

        idxs = dataset['collections'][selected_collection_key]['labels'][sensor_key]['idxs']
        idxs_limit_points = dataset['collections'][selected_collection_key]['labels'][sensor_key]['idxs_limit_points']
        sensor_idx = list(dataset['collections'][selected_collection_key]['labels'].keys()).index(sensor_key)

        # print('pointcloud_msg.height=' + str(pointcloud_msg.height))
        # print('pointcloud_msg.width=' + str(pointcloud_msg.width))

        points = []  # Build a list of points.
        for idx, point in enumerate(list(pc2.read_points(pointcloud_msg))):

            # Colorize points according to if they are labels or not
            if idx in idxs_limit_points:  # Dark green.
                r, g, b = 50, 70, 20
            elif idx in idxs:  # Green.
                r, g, b = 100, 220, 20
            else:  # Gray.
                r, g, b = 186, 189, 182

            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

            point_color = [point[0], point[1], point[2], idx, rgb, sensor_idx]
            points.append(point_color)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('idx', 12, PointField.FLOAT32, 1),
                  PointField('rgb', 20, PointField.UINT32, 1),
                  PointField('sensor_idx', 24, PointField.FLOAT32, 1)]
        pointcloud_msg_labeled = pc2.create_cloud(pointcloud_msg.header, fields, points)

        graphics['ros']['sensors'][sensor_key]['PubPointCloud'].publish(pointcloud_msg_labeled)

    # Create a new marker array which contains only the marker related to the selected collection
    # Publish the robot_mesh_
    marker_array_3 = MarkerArray()
    for marker in graphics['ros']['MarkersLaserBeams'].markers:
        prefix = marker.header.frame_id[:3]
        if prefix == 'c' + str(selected_collection_key) + '_':
            marker_array_3.markers.append(marker)
            marker_array_3.markers[-1].action = Marker.ADD
        elif not previous_selected_collection_key == selected_collection_key and prefix == 'c' + str(
                previous_selected_collection_key) + '_':
            marker_array_3.markers.append(marker)
            marker_array_3.markers[-1].action = Marker.DELETE

    graphics['ros']['PubLaserBeams'].publish(
        graphics['ros']['MarkersLaserBeams'])

    # Publish Annotated images
    for sensor_key, sensor in sensors.items():
        # if sensor['msg_type'] == 'Image':
        if sensor['modality'] == 'rgb':
            if args['show_images']:
                collection = collections[selected_collection_key]
                image = copy.deepcopy(getCvImageFromCollectionSensor(
                    selected_collection_key, sensor_key, dataset))
                width = collection['data'][sensor_key]['width']
                height = collection['data'][sensor_key]['height']
                diagonal = math.sqrt(width ** 2 + height ** 2)
                cm = graphics['pattern']['colormap']

                # Draw projected points (as dots)
                # for idx, point in enumerate(collection['labels'][sensor_key]['idxs_projected']):
                #     x = int(round(point['x']))
                #     y = int(round(point['y']))
                #     color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                #     cv2.line(image, (x, y), (x, y), color, int(6E-3 * diagonal))

                # Draw ground truth points (as squares)
                for idx, point in enumerate(collection['labels'][sensor_key]['idxs']):
                    x = int(round(point['x']))
                    y = int(round(point['y']))
                    color = (cm[idx, 2] * 255, cm[idx, 1]
                             * 255, cm[idx, 0] * 255)
                    drawSquare2D(image, x, y, int(8E-3 * diagonal),
                                 color=color, thickness=2)

                # Draw initial projected points (as crosses)
                # for idx, point in enumerate(collection['labels'][sensor_key]['idxs_initial']):
                #     x = int(round(point['x']))
                #     y = int(round(point['y']))
                #     color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                #     drawCross2D(image, x, y, int(8E-3 * diagonal), color=color, thickness=1)

                msg = CvBridge().cv2_to_imgmsg(image, "bgr8")

                msg.header.frame_id = 'c' + \
                    selected_collection_key + '_' + sensor['parent']
                graphics['collections'][sensor_key]['publisher'].publish(msg)

                # Publish camera info message
                camera_info_msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                                      sensor['camera_info'])
                camera_info_msg.header.frame_id = msg.header.frame_id
                graphics['collections'][sensor_key]['publisher_camera_info'].publish(
                    camera_info_msg)

        if sensor['modality'] == 'depth':
            if args['show_images']:

                # Shortcut variables
                collection = collections[selected_collection_key]
                clicked_sensor_points = clicked_points[selected_collection_key][sensor_key]['points']

                # Create image to draw on top
                image = getCvImageFromDictionaryDepth(collection['data'][sensor_key])
                gui_image = normalizeDepthImage(image, max_value=5)
                gui_image = drawLabelsOnImage(collection['labels'][sensor_key], gui_image)

                if not clicked_points[selected_collection_key][sensor_key]['valid_polygon']:
                    # Draw a cross for each point
                    for point in clicked_sensor_points:
                        drawSquare2D(gui_image, point['x'], point['y'], size=7, color=(50, 190, 0), thickness=2)

                    # Draw a line segment for each pair of consecutive points
                    for point_start, point_end in zip(clicked_sensor_points[:-1], clicked_sensor_points[1:]):
                        cv2.line(gui_image, pt1=(point_start['x'], point_start['y']),
                                 pt2=(point_end['x'], point_end['y']),
                                 color=(0, 0, 255), thickness=2)

                msg = CvBridge().cv2_to_imgmsg(gui_image, "passthrough")

                msg.header.frame_id = 'c' + selected_collection_key + '_' + sensor['parent']
                graphics['collections'][sensor_key]['publisher'].publish(msg)

                # Publish camera info message
                camera_info_msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                                      sensor['camera_info'])
                camera_info_msg.header.frame_id = msg.header.frame_id
                graphics['collections'][sensor_key]['publisher_camera_info'].publish(
                    camera_info_msg)

            # elif sensor['msg_type'] == 'LaserScan':
        elif sensor['modality'] == 'lidar2d':
            pass
            # elif sensor['msg_type'] == 'PointCloud2':
        elif sensor['modality'] == 'lidar3d':
            pass
        else:
            pass

    graphics['ros']['Rate'].sleep()
