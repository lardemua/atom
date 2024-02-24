#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib
import copy
import math
import os
import pprint

import networkx as nx
import matplotlib.pyplot as plt

# 3rd-party
import colorama
import cv2
import atom_core.ros_numpy

# import numpy as np
import rospy
import numpy as np
import tf
import tf2_ros
from atom_calibration.dataset_playback.depth_manual_labeling import drawLabelsOnImage, normalizeDepthImage
from cv2 import STEREO_BM_PREFILTER_NORMALIZED_RESPONSE
from atom_calibration.dataset_playback.visualization import getCvDepthImageFromCollectionSensor
from atom_core.cache import Cache
from rospy_message_converter import message_converter
from atom_core.rospy_urdf_to_rviz_converter import urdfToMarkerArray
from std_msgs.msg import Header, ColorRGBA
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import Image, sensor_msgs, CameraInfo, geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion, Transform, TransformStamped
from cv_bridge import CvBridge
from colorama import Style, Fore
from matplotlib import cm

# own packages
from atom_core.drawing import drawSquare2D, drawCross2D
from atom_core.naming import generateLabeledTopic, generateName
from atom_core.system import execute
from atom_core.config_io import uriReader
from atom_core.xacro_io import readXacroFile
from atom_core.dataset_io import (getCvImageFromDictionary, getCvImageFromDictionaryDepth,
                                  getPointCloudMessageFromDictionary, genCollectionPrefix)
from atom_calibration.calibration.objective_function import *

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


@Cache(args_to_ignore=['dataset'])
def getCvImageFromCollectionSensor(collection_key, sensor_key, dataset):
    dictionary = dataset['collections'][collection_key]['data'][sensor_key]
    return getCvImageFromDictionary(dictionary)


def createPatternMarkers(frame_id, ns, collection_key, pattern_key, now, dataset, graphics):
    # print('Creating pattern markers for pattern ' + pattern_key)
    markers = MarkerArray()

    # Draw pattern frame lines_sampled (top, left, right, bottom)
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-frame_sampled', id=0, frame_locked=True,
                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.01, y=0.01, z=0.01),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0))

    pts = []
    pts.extend(dataset['patterns'][pattern_key]['frame']['lines_sampled']['left'])
    pts.extend(dataset['patterns'][pattern_key]['frame']['lines_sampled']['right'])
    pts.extend(dataset['patterns'][pattern_key]['frame']['lines_sampled']['top'])
    pts.extend(dataset['patterns'][pattern_key]['frame']['lines_sampled']['bottom'])
    for pt in pts:
        marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))

    markers.markers.append(marker)

    # Draw corners
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-corners', id=0, frame_locked=True,
                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.02, y=0.02, z=0.02),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0))

    for idx_corner, pt in enumerate(dataset['patterns'][pattern_key]['corners']):
        marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))
        marker.colors.append(ColorRGBA(r=graphics['patterns'][pattern_key]['colormap'][idx_corner, 0],
                                       g=graphics['patterns'][pattern_key]['colormap'][idx_corner, 1],
                                       b=graphics['patterns'][pattern_key]['colormap'][idx_corner, 2], a=1))

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

    # Draw the mesh
    m = Marker(header=Header(frame_id=frame_id, stamp=now),
               ns=ns + '-mesh', id=0, frame_locked=True,
               type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(0),
               pose=Pose(position=Point(x=0, y=0, z=0),
                         orientation=Quaternion(x=0, y=0, z=0, w=1)),
               scale=Vector3(x=1.0, y=1.0, z=1.0),
               color=ColorRGBA(r=1, g=1, b=1, a=1))

    mesh_file, _, _ = uriReader(dataset['calibration_config']['calibration_patterns'][pattern_key]['mesh_file'])
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
    graphics = {'collections': {}, 'patterns': {}, 'ros': {'sensors': {}, 'patterns': {}}, 'args': args}

    # Parse xacro description file
    description_file, _, _ = uriReader(dataset['calibration_config']['description_file'])
    rospy.loginfo('Reading description file ' + description_file + '...')
    # TODO not sure this should be done because of the use_tfs functionality ...
    xml_robot = readXacroFile(description_file)

    # Initialize ROS stuff
    rospy.init_node("calibrate")
    # graphics['ros']['tf_broadcaster'] = tf.TransformBroadcaster()
    graphics['ros']['tf_broadcaster'] = tf2_ros.TransformBroadcaster()

    rospy.sleep(0.2)  # Sleep a little to make sure the time.now() returns a correct time.
    now = rospy.Time.now()

    graphics['ros']['publisher_models'] = rospy.Publisher('~robot_meshes', MarkerArray, queue_size=0, latch=True)
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
    for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
        graphics['patterns'][pattern_key] = {}
        graphics['patterns'][pattern_key]['colormap'] = cm.gist_rainbow(
            np.linspace(0, 1, pattern['dimension']['x'] * pattern['dimension']['y']))

    graphics['collections']['colormap'] = cm.prism(np.linspace(0, 1, len(dataset['collections'].keys())))
    # graphics['collections']['colormap'] = cm.Wistia(np.linspace(0, 1, len(dataset['collections'].keys())))
    for idx, collection_key in enumerate(sorted(dataset['collections'].keys())):
        graphics['collections'][str(collection_key)] = {'color': graphics['collections']['colormap'][idx, :]}

    # color_map_sensors = cm.gist_rainbow(np.linspace(0, 1, len(dataset['sensors'].keys())))
    # for idx, sensor_key in enumerate(sorted(dataset['sensors'].keys())):
    #     dataset['sensors'][str(sensor_key)]['color'] = color_map_sensors[idx, :]

    # ----------------------------------------------------------------------------------------
    # Create 2D Labels  (only for rgb and depth)
    # Note: Republish a new image at every visualization, because the labels must redrawn as they change position
    # ----------------------------------------------------------------------------------------
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():

            # check if sensor detects and of the patterns
            flag_detects_at_least_one_pattern = False
            for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
                if collection['labels'][pattern_key][sensor_key]['detected']:
                    flag_detects_at_least_one_pattern = True

            # If in this collection, sensor did not detect any of the patterns, continue
            if flag_detects_at_least_one_pattern == False:
                continue

            if sensor['modality'] in ['rgb', 'depth']:
                graphics['collections'][collection_key][sensor_key] = {}

                # Image msg setup.
                labeled_topic = generateLabeledTopic(dataset['calibration_config']['sensors'][sensor_key]['topic_name'],
                                                     collection_key=collection_key, type='2d')
                graphics['collections'][collection_key][sensor_key]['publisher'] = rospy.Publisher(
                    labeled_topic, sensor_msgs.msg.Image, queue_size=0, latch=True)

                # Camera info msg setup.
                labeled_topic = generateLabeledTopic(dataset['calibration_config']['sensors'][sensor_key]['topic_name'],
                                                     collection_key=collection_key, type='2d', suffix='/camera_info')
                # topic_name = '~c' + str(collection_key) + '/' + str(sensor_key) + '/camera_info'
                graphics['collections'][collection_key][sensor_key]['publisher_camera_info'] = rospy.Publisher(
                    labeled_topic, sensor_msgs.msg.CameraInfo, queue_size=0, latch=True)

    # ----------------------------------------------------------------------------------------
    # Create 3D Labels  (only for lidar2d, lidar3d and depth)
    # ----------------------------------------------------------------------------------------
    for sensor_key, sensor in dataset['sensors'].items():
        markers = MarkerArray()
        for collection_key, collection in dataset['collections'].items():

            # check if sensor detects and of the patterns
            flag_detects_at_least_one_pattern = False
            for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
                if collection['labels'][pattern_key][sensor_key]['detected']:
                    flag_detects_at_least_one_pattern = True

            # If in this collection, sensor did not detect any of the patterns, continue
            if flag_detects_at_least_one_pattern == False:
                continue

            for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
                if sensor['modality'] == 'lidar2d':
                    frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                    ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                    type=Marker.POINTS, action=Marker.ADD, lifetime=rospy.Duration(0),
                                    pose=Pose(position=Point(x=0, y=0, z=0),
                                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                    scale=Vector3(x=0.03, y=0.03, z=0),
                                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                    g=graphics['collections'][collection_key]['color'][1],
                                                    b=graphics['collections'][collection_key]['color'][2], a=1.0)
                                    )

                    # Get laser points that belong to the chessboard (labelled)
                    idxs = collection['labels'][sensor_key]['idxs']
                    rhos = [collection['data'][sensor_key]['ranges'][idx] for idx in idxs]
                    thetas = [collection['data'][sensor_key]['angle_min'] +
                              collection['data'][sensor_key]['angle_increment'] * idx for idx in idxs]

                    for idx, (rho, theta) in enumerate(zip(rhos, thetas)):
                        marker.points.append(Point(x=rho * math.cos(theta), y=rho * math.sin(theta), z=0))

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
                    for edge_idx in collection['labels'][sensor_key]['edge_idxs']:  # add edge points
                        p = Point()
                        p.x = rhos[edge_idx] * math.cos(thetas[edge_idx])
                        p.y = rhos[edge_idx] * math.sin(thetas[edge_idx])
                        p.z = 0
                        marker.points.append(p)
                    markers.markers.append(copy.deepcopy(marker))

                # if sensor['msg_type'] == 'PointCloud2':  # -------- Publish the velodyne data ----------------------------
                elif sensor['modality'] == 'lidar3d':
                    # Add labelled points to the marker
                    frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                    ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                    type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                    pose=Pose(position=Point(x=0, y=0, z=0),
                                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                    scale=Vector3(x=0.02, y=0.02, z=0.02),
                                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                    g=graphics['collections'][collection_key]['color'][1],
                                                    b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                    )

                    points = getPointsInSensorAsNPArray(collection_key, pattern_key, sensor_key, 'idxs', dataset)
                    for idx in range(0, points.shape[1]):
                        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                    markers.markers.append(copy.deepcopy(marker))

                    # Add limit points to the marker, this time with larger spheres
                    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                    ns=str(collection_key) + '-' + str(sensor_key) + '-limit_points', id=0,
                                    frame_locked=True,
                                    type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                    pose=Pose(position=Point(x=0, y=0, z=0),
                                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                    scale=Vector3(x=0.07, y=0.07, z=0.07),
                                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                    g=graphics['collections'][collection_key]['color'][1],
                                                    b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                    )

                    points = getPointsInSensorAsNPArray(collection_key, pattern_key,
                                                        sensor_key, 'idxs_limit_points', dataset)
                    for idx in range(0, points.shape[1]):
                        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                    markers.markers.append(copy.deepcopy(marker))

                # Setup visualization for depth
                elif sensor['modality'] == 'depth':  # -------- Publish the depth  ----------------------------
                    # Add labelled points to the marker
                    frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                    ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                    type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                    pose=Pose(position=Point(x=0, y=0, z=0),
                                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                    scale=Vector3(x=0.01, y=0.01, z=0.01),
                                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                    g=graphics['collections'][collection_key]['color'][1],
                                                    b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                    )

                    points = getPointsInDepthSensorAsNPArray(collection_key, pattern_key, sensor_key, 'idxs', dataset)
                    for idx in range(0, points.shape[1]):
                        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                    markers.markers.append(copy.deepcopy(marker))

                    # Add limit points to the marker, this time with larger spheres
                    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                    ns=str(collection_key) + '-' + str(sensor_key) + '-limit_points', id=0,
                                    frame_locked=True,
                                    type=Marker.CUBE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                    pose=Pose(position=Point(x=0, y=0, z=0),
                                              orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                    scale=Vector3(x=0.05, y=0.05, z=0.05),
                                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                    g=graphics['collections'][collection_key]['color'][1],
                                                    b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                    )

                    points = getPointsInDepthSensorAsNPArray(
                        collection_key, pattern_key, sensor_key, 'idxs_limit_points', dataset)
                    for idx in range(0, points.shape[1]):
                        marker.points.append(Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                    markers.markers.append(copy.deepcopy(marker))

            graphics['ros']['sensors'][sensor_key] = {}
            graphics['ros']['sensors'][sensor_key]['MarkersLabeled'] = markers
            labeled_topic = generateLabeledTopic(dataset['sensors'][sensor_key]['topic'], type='3d')
            graphics['ros']['sensors'][sensor_key]['PubLabeled'] = rospy.Publisher(
                labeled_topic, MarkerArray, queue_size=0, latch=True)

    # -----------------------------------------------------------------------------------------------------
    # -------- Robot meshes
    # -----------------------------------------------------------------------------------------------------

    # Evaluate for each link if it may move or not (movable or immovable), to see if it needs to be drawn for each
    # collection. This is done by comparing the several transformations from the world_link to the <link> obtained
    # from the collections.
    immovable_links = []
    movable_links = []
    for link in xml_robot.links:  # cycle all links

        # print(dataset['calibration_config']['world_link'] + ' to ' + link.name + ':')
        first_time = True
        for collection_key, collection in dataset['collections'].items():
            transform = getTransform(dataset['calibration_config']['world_link'], link.name,
                                     collection['transforms'])
            # print('Collection ' + collection_key + ': ')
            if first_time:
                first_time = False
                transform_first_time = transform
            elif not np.array_equal(transform_first_time, transform):
                movable_links.append(link.name)
                break

        if link.name not in movable_links:
            immovable_links.append(link.name)

    print('immovable links are: ' + str(immovable_links))
    print('movable links are: ' + str(movable_links))

    # Check whether the robot is static, in the sense that all of its joints are fixed. If so, for efficiency purposes,
    # only one robot mesh (from the selected collection) is published.
    if args['all_joints_fixed']:  # assume the robot is static
        all_joints_fixed = True
        print('Robot is assumed to have all joints fixed.')
    else:  # run automatic detection
        all_joints_fixed = True
        for joint in xml_robot.joints:
            if not joint.type == 'fixed':
                print('Robot has at least joint ' + joint.name + ' non fixed. Will render all collections')
                all_joints_fixed = False
                break

    markers = MarkerArray()
    if all_joints_fixed:  # render a single robot mesh
        print('Robot has all joints fixed. Will render only collection ' + selected_collection_key)
        rgba = [.5, .5, .5, 1]  # best color we could find
        m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(selected_collection_key, ''),
                              namespace='immovable',
                              rgba=rgba)
        markers.markers.extend(m.markers)

        if args['initial_pose_ghost']:  # add a ghost (low alpha) robot marker at the initial pose
            rgba = [.1, .1, .8, 0.1]  # best color we could find
            m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(selected_collection_key, ''),
                                  frame_id_suffix=generateName('', suffix='ini'),
                                  namespace=generateName('immovable', suffix='ini'),
                                  rgba=rgba)
            markers.markers.extend(m.markers)

    else:  # render robot meshes for all collections
        print('Robot has some dynamic joints. Will use advanced rendering ...')

        # Draw immovable links
        rgba = None  # Immovable links are always drawn with their original color
        m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(selected_collection_key, ''),
                              namespace='immovable',
                              rgba=rgba, skip_links=movable_links, alpha=1.0)  # Immovable do not need alpha
        markers.markers.extend(m.markers)

        if args['initial_pose_ghost']:  # add a ghost robot marker at the initial pose
            rgba = [.1, .8, .1, 0.1]  # ipg are drawn in transparent green

        # Draw movable links
        for collection_key, collection in dataset['collections'].items():

            if args['draw_per_collection_colors']:
                rgba = graphics['collections'][collection_key]['color']
            else:
                rgba = None

            m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                  namespace=collection_key,
                                  rgba=rgba, skip_links=immovable_links,
                                  verbose=False, alpha=args['draw_alpha'])
            markers.markers.extend(m.markers)

            if args['initial_pose_ghost']:  # add a ghost (low alpha) robot marker at the initial pose
                rgba = [.1, .8, .1, 0.1]  # best color we could find
                # Draw immovable links
                m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                      frame_id_suffix=generateName('', suffix='ini'),
                                      namespace=generateName('immovable', suffix='ini'),
                                      rgba=rgba, skip_links=movable_links)
                markers.markers.extend(m.markers)

                # Draw movable links
                for collection_key, collection in dataset['collections'].items():
                    m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                          frame_id_suffix=generateName('', suffix='ini'),
                                          namespace=generateName(collection_key, suffix='ini'),
                                          rgba=rgba, skip_links=immovable_links)
                    markers.markers.extend(m.markers)

    graphics['ros']['RobotMeshMarkers'] = markers

    # -----------------------------------------------------------------------------------------------------
    # -------- Publish the pattern data
    # -----------------------------------------------------------------------------------------------------

    markers = MarkerArray()
    for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
        if pattern['fixed'] and pattern['parent_link'] == dataset['calibration_config']['world_link']:
            # Draw single pattern for selected collection key
            frame_id = generateName(pattern['link'], prefix='c' + selected_collection_key)
            ns = selected_collection_key + '_' + pattern_key
            pattern_markers = createPatternMarkers(frame_id, ns, selected_collection_key, pattern_key,
                                                   now, dataset, graphics)
            markers.markers.extend(pattern_markers.markers)
        else:  # Draw a pattern per collection
            for idx, (collection_key, collection) in enumerate(dataset['collections'].items()):
                frame_id = generateName(pattern['link'],
                                        prefix='c' + collection_key)
                ns = collection_key + '_' + pattern_key
                collection_markers = createPatternMarkers(
                    frame_id, ns, collection_key, pattern_key, now, dataset, graphics)
                markers.markers.extend(collection_markers.markers)

    graphics['ros']['MarkersPattern'] = markers
    graphics['ros']['PubPattern'] = rospy.Publisher(
        '~patterns', MarkerArray, queue_size=0, latch=True)

    # Create LaserBeams Publisher -----------------------------------------------------------
    # This one is recomputed every time in the objective function, so just create the generic properties.
    markers = MarkerArray()

    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():

            # check if sensor detects and of the patterns
            flag_detects_at_least_one_pattern = False
            for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
                if collection['labels'][pattern_key][sensor_key]['detected']:
                    flag_detects_at_least_one_pattern = True

            # If in this collection, sensor did not detect any of the patterns, continue
            if flag_detects_at_least_one_pattern == False:
                continue

            # if sensor['msg_type'] == 'LaserScan' or sensor['msg_type'] == 'PointCloud2':
            if sensor['modality'] == 'lidar2d' or sensor['modality'] == 'lidar3d':
                frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.LINE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                scale=Vector3(x=0.001, y=0, z=0),
                                color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                g=graphics['collections'][collection_key]['color'][1],
                                                b=graphics['collections'][collection_key]['color'][2], a=1.0)
                                )
                markers.markers.append(marker)

    graphics['ros']['MarkersLaserBeams'] = markers
    graphics['ros']['PubLaserBeams'] = rospy.Publisher('~LaserBeams', MarkerArray, queue_size=0, latch=True)

    # Create Miscellaneous MarkerArray -----------------------------------------------------------
    markers = MarkerArray()

    # Text signaling the anchored sensor
    for _sensor_key, sensor in dataset['sensors'].items():
        if _sensor_key == dataset['calibration_config']['anchored_sensor']:
            marker = Marker(header=Header(frame_id=str(_sensor_key), stamp=now),
                            ns=str(_sensor_key), id=0, frame_locked=True,
                            type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, lifetime=rospy.Duration(0),
                            pose=Pose(position=Point(x=0, y=0, z=0.2), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                            scale=Vector3(x=0.0, y=0.0, z=0.1),
                            color=ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0), text='Anchored')

            markers.markers.append(marker)

    graphics['ros']['MarkersMiscellaneous'] = markers
    graphics['ros']['PubMiscellaneous'] = rospy.Publisher('~Miscellaneous', MarkerArray, queue_size=0, latch=True)
    # Publish only once in latched mode
    graphics['ros']['PubMiscellaneous'].publish(graphics['ros']['MarkersMiscellaneous'])
    graphics['ros']['Rate'] = rospy.Rate(10)
    graphics['ros']['Rate'] = rospy.Rate(10)
    graphics['ros']['Counter'] = 0  # tfs need to be published at high frequencies. On the other hand, robot markers
    # should be published at low frequencies. This counter will serve to control this mechanism.

    return graphics


def visualizationFunction(models):
    # print(Fore.RED + 'Visualization function called.' + Style.RESET_ALL)
    # Get the data from the meshes
    dataset = models['dataset']
    args = models['args']
    collections = models['dataset']['collections']
    sensors = models['dataset']['sensors']
    patterns = models['dataset']['patterns']
    config = models['dataset']['calibration_config']
    graphics = models['graphics']

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

        if args['initial_pose_ghost']:
            parent = config['world_link']
            child = generateName(config['world_link'], prefix='c' + collection_key, suffix='ini')
            transform = TransformStamped(header=Header(frame_id=parent, stamp=now),
                                         child_frame_id=child,
                                         transform=Transform(translation=Vector3(x=0, y=0, z=0),
                                                             rotation=Quaternion(x=0, y=0, z=0, w=1)))
            transfoms.append(transform)

        # Publish all current transforms
        for transform_key, transform in collection['transforms'].items():
            parent = generateName(transform['parent'], prefix='c' + collection_key)
            child = generateName(transform['child'], prefix='c' + collection_key)
            x, y, z = transform['trans']
            qx, qy, qz, qw = transform['quat']
            transform = TransformStamped(header=Header(frame_id=parent, stamp=now),
                                         child_frame_id=child,
                                         transform=Transform(translation=Vector3(x=x, y=y, z=z),
                                                             rotation=Quaternion(x=qx, y=qy, z=qz, w=qw)))
            transfoms.append(transform)

        if args['initial_pose_ghost']:  # Publish robot meshes at initial pose.
            for transform_key, transform in collection[generateName('transforms', suffix='ini')].items():
                parent = generateName(transform['parent'], prefix='c' + collection_key)
                child = generateName(transform['child'], prefix='c' + collection_key)
                x, y, z = transform['trans']
                qx, qy, qz, qw = transform['quat']
                transform = TransformStamped(header=Header(frame_id=parent, stamp=now),
                                             child_frame_id=child,
                                             transform=Transform(translation=Vector3(x=x, y=y, z=z),
                                                                 rotation=Quaternion(x=qx, y=qy, z=qz, w=qw)))
                transfoms.append(transform)

    graphics['ros']['tf_broadcaster'].sendTransform(transfoms)

    # print("graphics['ros']['Counter'] = " + str(graphics['ros']['Counter']))
    if graphics['ros']['Counter'] < 1:
        graphics['ros']['Counter'] += 1
        return None
    else:
        graphics['ros']['Counter'] = 0

    # Update markers stamp, so that rviz uses newer transforms to compute their poses.
    for marker in graphics['ros']['RobotMeshMarkers'].markers:
        marker.header.stamp = now

    # Publish the meshes
    graphics['ros']['publisher_models'].publish(graphics['ros']['RobotMeshMarkers'])

    # Publish patterns
    for marker in graphics['ros']['MarkersPattern'].markers:
        marker.header.stamp = now
    graphics['ros']['PubPattern'].publish(graphics['ros']['MarkersPattern'])

    # TODO update markers
    # Publish Labelled Data
    for sensor_key, sensor in sensors.items():
        for marker in graphics['ros']['sensors'][sensor_key]['MarkersLabeled'].markers:
            marker.header.stamp = now
        graphics['ros']['sensors'][sensor_key]['PubLabeled'].publish(
            graphics['ros']['sensors'][sensor_key]['MarkersLabeled'])

    # Publish Laser Beams
    for marker in graphics['ros']['MarkersLaserBeams'].markers:
        marker.header.stamp = now
    graphics['ros']['PubLaserBeams'].publish(graphics['ros']['MarkersLaserBeams'])

    # ---------------------------------------------------------------------------------
    # Publish 2D labels
    # ---------------------------------------------------------------------------------
    if args['show_images']:

        for collection_key, collection in collections.items():
            for sensor_key, sensor in sensors.items():

                # check if sensor detects any of the patterns
                flag_detects_at_least_one_pattern = False
                for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():
                    if collection['labels'][pattern_key][sensor_key]['detected']:
                        flag_detects_at_least_one_pattern = True

                # If in this collection, sensor did not detect any of the patterns, continue
                if flag_detects_at_least_one_pattern == False:
                    continue

                if sensor['modality'] == 'rgb':
                    image = copy.deepcopy(getCvImageFromCollectionSensor(collection_key, sensor_key, dataset))
                    width = collection['data'][sensor_key]['width']
                    height = collection['data'][sensor_key]['height']
                    diagonal = math.sqrt(width ** 2 + height ** 2)

                    for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():

                        # check if sensor detects pattern
                        if not collection['labels'][pattern_key][sensor_key]['detected']:
                            continue

                        cm = graphics['patterns'][pattern_key]['colormap']
                        # Draw projected points (as dots)
                        for idx, point in enumerate(collection['labels'][pattern_key][sensor_key]['idxs_projected']):
                            x = int(round(point['x']))
                            y = int(round(point['y']))
                            color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                            cv2.line(image, (x, y), (x, y), color, int(6E-3 * diagonal))

                        # Draw ground truth points (as squares)
                        for idx, point in enumerate(collection['labels'][pattern_key][sensor_key]['idxs']):
                            x = int(round(point['x']))
                            y = int(round(point['y']))
                            color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                            drawSquare2D(image, x, y, int(8E-3 * diagonal), color=color, thickness=2)

                        # Draw initial projected points (as crosses)
                        for idx, point in enumerate(collection['labels'][pattern_key][sensor_key]['idxs_initial']):
                            x = int(round(point['x']))
                            y = int(round(point['y']))
                            color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                            drawCross2D(image, x, y, int(8E-3 * diagonal), color=color, thickness=1)

                    msg = CvBridge().cv2_to_imgmsg(image, "bgr8")

                    msg.header.frame_id = 'c' + collection_key + '_' + sensor['parent']
                    graphics['collections'][collection_key][sensor_key]['publisher'].publish(msg)

                    # Publish camera info message
                    camera_info_msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                                          sensor['camera_info'])
                    camera_info_msg.header.frame_id = msg.header.frame_id
                    graphics['collections'][collection_key][sensor_key]['publisher_camera_info'].publish(
                        camera_info_msg)

                elif sensor['modality'] == 'depth':
                    # Shortcut variables
                    collection = collections[collection_key]
                    image = copy.deepcopy(
                        getCvDepthImageFromCollectionSensor(collection_key, sensor_key, dataset, scale=10000.0))

                    width = collection['data'][sensor_key]['width']
                    height = collection['data'][sensor_key]['height']
                    diagonal = math.sqrt(width ** 2 + height ** 2)/2
                    # print(width, height)

                    for pattern_key, pattern in dataset['calibration_config']['calibration_patterns'].items():

                        # check if sensor detects pattern
                        if not collection['labels'][pattern_key][sensor_key]['detected']:
                            continue

                        idxs = collection['labels'][pattern_key][sensor_key]['idxs']
                        idxs_limit_points = collection['labels'][pattern_key][sensor_key]['idxs_limit_points']
                        gui_image = np.zeros((height, width, 3), dtype=np.uint8)
                        max_value = 5
                        gui_image[:, :, 0] = image / max_value * 255
                        gui_image[:, :, 1] = image / max_value * 255
                        gui_image[:, :, 2] = image / max_value * 255

                        for idx in idxs:
                            # convert from linear idx to x_pix and y_pix indices.
                            y = int(idx / width)
                            x = int(idx - y * width)
                            cv2.line(gui_image, (x, y), (x, y), (0, 200, 255), 3)
                        for idx in idxs_limit_points:
                            # convert from linear idx to x_pix and y_pix indices.
                            y = int(idx / width)
                            x = int(idx - y * width)
                            drawSquare2D(gui_image, x, y, int(8E-3 * diagonal), (255, 0, 200), thickness=1)

                        # Draw projected points (as dots)
                        for idx, point in enumerate(collection['labels'][pattern_key][sensor_key]['idxs_projected']):
                            x = int(round(point['x']))
                            y = int(round(point['y']))
                            if x < width - 1 and y < height - 1:
                                cv2.line(gui_image, (x, y), (x, y), (0, 0, 255), 3)

                    msg = CvBridge().cv2_to_imgmsg(gui_image, "passthrough")

                    msg.header.frame_id = 'c' + collection_key + '_' + sensor['parent']
                    graphics['collections'][collection_key][sensor_key]['publisher'].publish(msg)

                    # Publish camera info message
                    camera_info_msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                                          sensor['camera_info'])
                    camera_info_msg.header.frame_id = msg.header.frame_id
                    graphics['collections'][collection_key][sensor_key]['publisher_camera_info'].publish(
                        camera_info_msg)

    graphics['ros']['Rate'].sleep()
