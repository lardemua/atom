"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

import copy
import math
import os
import struct
from re import I

# 3rd-party
import colorama
import cv2
import cv_bridge
import numpy as np
import ros_numpy
# import numpy as np  # TODO Eurico, line  fails if I don't do this
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
import visualization_msgs.msg
from atom_calibration.calibration.objective_function import *
from atom_calibration.collect.label_messages import *
from atom_calibration.dataset_playback.depth_manual_labeling import (
    drawLabelsOnImage, normalizeDepthImage)
from atom_core.cache import Cache
from atom_core.config_io import execute, readXacroFile, uriReader
from atom_core.dataset_io import (genCollectionPrefix,
                                  getCvImageFromDictionary,
                                  getCvImageFromDictionaryDepth,
                                  getMsgAndCvImageFromDictionaryDepth,
                                  getPointCloudMessageFromDictionary)
from atom_core.drawing import drawCross2D, drawSquare2D
from atom_core.naming import generateName
from atom_core.rospy_urdf_to_rviz_converter import urdfToMarkerArray
from colorama import Fore, Style
from cv_bridge import CvBridge
from geometry_msgs.msg import (Point, Pose, Quaternion, Transform,
                               TransformStamped, Vector3)
from matplotlib import cm
from rospy_message_converter import message_converter
from scipy.spatial import distance
from sensor_msgs.msg import (CameraInfo, Image, PointCloud2, PointField,
                             geometry_msgs, sensor_msgs)
from std_msgs.msg import ColorRGBA, Header, UInt8MultiArray
from urdf_parser_py.urdf import URDF
# stdlib
from visualization_msgs.msg import Marker, MarkerArray

# own packages

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

# although this function is somewhere else, in the other place it uses the dataset as cache...


def getPointsInSensorAsNPArray_local(_collection_key, _sensor_key, _label_key, _dataset):
    cloud_msg = getPointCloudMessageFromDictionary(
        _dataset['collections'][_collection_key]['data'][_sensor_key])
    idxs = _dataset['collections'][_collection_key]['labels'][_sensor_key][_label_key]
    pc = ros_numpy.numpify(cloud_msg)[idxs]
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
    graphics = {'collections': {}, 'sensors': {},
                'pattern': {}, 'ros': {}, 'args': args}

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
    rospy.sleep(0.2)
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

    # Create image publishers ----------------------------------------------------------
    # We need to republish a new image at every visualization
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['modality'] == 'rgb':
            msg_type = sensor_msgs.msg.Image
            topic = dataset['calibration_config']['sensors'][sensor_key]['topic_name']
            topic_name = topic + '/labeled'
            graphics['collections'][str(sensor_key)] = {'publisher': rospy.Publisher(
                topic_name, msg_type, queue_size=0, latch=True)}
            print('Created image publisher')
            msg_type = sensor_msgs.msg.CameraInfo
            topic_name = str(sensor_key) + '/camera_info'
            graphics['collections'][str(sensor_key)]['publisher_camera_info'] = \
                rospy.Publisher(topic_name, msg_type, queue_size=0, latch=True)

        if sensor['modality'] == 'depth':
            msg_type = sensor_msgs.msg.Image
            topic = dataset['calibration_config']['sensors'][sensor_key]['topic_name']
            topic_name = topic + '/labeled'
            graphics['collections'][str(sensor_key)] = {'publisher': rospy.Publisher(
                topic_name, msg_type, queue_size=0, latch=True)}
            print('Created image publisher')
            msg_type = sensor_msgs.msg.CameraInfo
            topic_name = str(sensor_key) + '/camera_info'
            graphics['collections'][str(sensor_key)]['publisher_camera_info'] = \
                rospy.Publisher(topic_name, msg_type, queue_size=0, latch=True)
    # Create Labeled and Unlabeled Data publishers ----------------------------------------------------------
    markers = MarkerArray()

    lidar_data = []
    graphics['ros']['PubPointCloud'] = dict()
    for sensor_key, sensor in dataset['sensors'].items():
        if sensor['modality'] == 'lidar3d':
            graphics['ros']['PubPointCloud'][sensor_key] = \
                rospy.Publisher(str(sensor_key) + '/points',
                                PointCloud2, queue_size=0, latch=True)

        for collection_key, collection in dataset['collections'].items():
            # if not collection['labels'][str(sensor_key)]['detected']:  # not detected by sensor in collection
            #     continue

            # when the sensor has no label, the 'idxs_limit_points' does not exist!!!
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
            if sensor['modality'] == 'lidar3d':

                # Add labelled points to the marker
                frame_id = genCollectionPrefix(
                    collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(
                                    0),
                                pose=Pose(position=Point(
                                    x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                scale=Vector3(x=0.05, y=0.05, z=0.05),
                                color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                g=graphics['collections'][collection_key]['color'][1],
                                                b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                )

                points = getPointsInSensorAsNPArray(
                    collection_key, sensor_key, 'idxs', dataset)

                for idx in range(0, points.shape[1]):
                    marker.points.append(
                        Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                markers.markers.append(copy.deepcopy(marker))

                # Add limit points to the marker, this time with larger spheres
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key) + '-limit_points', id=0,
                                frame_locked=True,
                                type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(
                                    0),
                                pose=Pose(position=Point(x=0, y=0, z=0),
                                          orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                scale=Vector3(x=0.07, y=0.07, z=0.07),
                                color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                g=graphics['collections'][collection_key]['color'][1],
                                                b=graphics['collections'][collection_key]['color'][2], a=0.5)
                                )

                points = getPointsInSensorAsNPArray(
                    collection_key, sensor_key, 'idxs_limit_points', dataset)
                for idx in range(0, points.shape[1]):
                    marker.points.append(
                        Point(x=points[0, idx], y=points[1, idx], z=points[2, idx]))

                markers.markers.append(copy.deepcopy(marker))

                # Add 3D lidar data
                original_pointcloud_msg = getPointCloudMessageFromDictionary(
                    dataset['collections'][collection_key]['data'][sensor_key])
                final_pointcloud_msg = PointCloud2(header=Header(frame_id=frame_id, stamp=now),
                                                   height=original_pointcloud_msg.height,
                                                   width=original_pointcloud_msg.width,
                                                   fields=original_pointcloud_msg.fields,
                                                   is_bigendian=original_pointcloud_msg.is_bigendian,
                                                   point_step=original_pointcloud_msg.point_step,
                                                   row_step=original_pointcloud_msg.row_step,
                                                   data=original_pointcloud_msg.data,
                                                   is_dense=original_pointcloud_msg.is_dense)
                lidar_data.append(final_pointcloud_msg)

    graphics['ros']['MarkersLabeled'] = markers
    graphics['ros']['PointClouds'] = lidar_data
    graphics['ros']['PubLabeled'] = rospy.Publisher(
        '~labeled_data', MarkerArray, queue_size=0, latch=True)

    # -----------------------------------------------------------------------------------------------------
    # -------- Robot meshes
    # -----------------------------------------------------------------------------------------------------

    # Evaluate for each link if it may move or not (movable or immovalbe), to see if it needs to be drawn for each
    # collection. This is done by comparing the several transformations from the world_link to the <link> obtained
    # from the collections.
    immovable_links = []
    movable_links = []
    for link in xml_robot.links:  # cycle all links

        print(dataset['calibration_config']
              ['world_link'] + ' to ' + link.name + ':')
        first_time = True
        for collection_key, collection in dataset['collections'].items():
            transform = atom_core.atom.getTransform(dataset['calibration_config']['world_link'], link.name,
                                                    collection['transforms'])
            print('Collection ' + collection_key + ': ')
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
                print('Robot has at least joint ' + joint.name +
                      ' non fixed. Will render all collections')
                all_joints_fixed = False
                break

    markers = MarkerArray()
    if all_joints_fixed:  # render a single robot mesh
        print('Robot has all joints fixed. Will render only collection ' +
              selected_collection_key)
        rgba = [.5, .5, .5, 1]  # best color we could find
        m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(selected_collection_key, ''),
                              namespace='immovable',
                              rgba=rgba)
        markers.markers.extend(m.markers)

    else:  # render robot meshes for all collections
        print('Robot has some dynamic joints. Will use advanced rendering ...')

        # Draw immovable links
        rgba = [.5, .5, .5, 1]  # best color we could find
        m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(selected_collection_key, ''),
                              namespace='immovable',
                              rgba=rgba, skip_links=movable_links)
        markers.markers.extend(m.markers)

        # Draw movable links
        for collection_key, collection in dataset['collections'].items():
            rgba = graphics['collections'][collection_key]['color']
            rgba[3] = 0.2  # change the alpha
            m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                  namespace=collection_key,
                                  rgba=rgba, skip_links=immovable_links)
            markers.markers.extend(m.markers)

            # add a ghost (low alpha) robot marker at the initial pose
            if args['initial_pose_ghost']:
                rgba = [.1, .1, .8, 0.1]  # best color we could find
                # Draw immovable links
                m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                      frame_id_suffix=generateName(
                                          '', suffix='ini'),
                                      namespace=generateName(
                                          'immovabl', suffix='ini'),
                                      rgba=rgba, skip_links=movable_links)
                markers.markers.extend(m.markers)

                # Draw movable links
                for collection_key, collection in dataset['collections'].items():
                    m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                          frame_id_suffix=generateName(
                                              '', suffix='ini'),
                                          namespace=generateName(
                                              collection_key, suffix='ini'),
                                          rgba=rgba, skip_links=immovable_links)
                    markers.markers.extend(m.markers)

    graphics['ros']['robot_mesh_markers'] = markers

    # -----------------------------------------------------------------------------------------------------
    # -------- Publish the pattern data
    # -----------------------------------------------------------------------------------------------------
    # Draw single pattern for selected collection key
    if dataset['calibration_config']['calibration_pattern']['fixed']:
        frame_id = generateName(dataset['calibration_config']['calibration_pattern']['link'],
                                prefix='c' + selected_collection_key)
        ns = str(selected_collection_key)
        markers = createPatternMarkers(
            frame_id, ns, selected_collection_key, now, dataset, graphics)
    else:  # Draw a pattern per collection
        markers = MarkerArray()
        for idx, (collection_key, collection) in enumerate(dataset['collections'].items()):
            frame_id = generateName(dataset['calibration_config']['calibration_pattern']['link'],
                                    prefix='c' + collection_key)
            ns = str(collection_key)
            collection_markers = createPatternMarkers(
                frame_id, ns, collection_key, now, dataset, graphics)
            markers.markers.extend(collection_markers.markers)

    graphics['ros']['MarkersPattern'] = markers
    graphics['ros']['PubPattern'] = rospy.Publisher(
        '~patterns', MarkerArray, queue_size=0, latch=True)

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
    patterns = models['dataset']['patterns']
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
    for marker in graphics['ros']['robot_mesh_markers'].markers:
        marker.header.stamp = now

    # Publish the meshes
    #  graphics['ros']['publisher_models'].publish(graphics['ros']['robot_mesh_markers'])

    # Update timestamp for the patterns markers
    for marker in graphics['ros']['MarkersPattern'].markers:
        marker.header.stamp = now

    # Update timestamp for labeled markers
    for marker in graphics['ros']['MarkersLabeled'].markers:
        marker.header.stamp = now

    # Update timestamp for laser beams markers
    for marker in graphics['ros']['MarkersLaserBeams'].markers:
        marker.header.stamp = now

    # Update timestamp for pointcloud2 message
    for pointcloud_msg in graphics['ros']['PointClouds']:
        pointcloud_msg.header.stamp = now

    # print(graphics['ros']['PointClouds'])
    # Create a new pointcloud which contains only the points related to the selected collection
    for pointcloud_msg in graphics['ros']['PointClouds']:

        prefix = pointcloud_msg.header.frame_id.split('_')[0] + '_'
        sensor = pointcloud_msg.header.frame_id[len(prefix):]

        # prof. Miguel these two lines that you wrote wasted 2 hours of my life!
        # prefix = pointcloud_msg.header.frame_id[:3]
        # sensor = pointcloud_msg.header.frame_id[3:]

        # print('-----------------------------------------------------------')
        if prefix == 'c' + str(selected_collection_key) + '_':
            # change intensity channel according to the idxs
            points_collection = pc2.read_points(pointcloud_msg)
            gen_points = list(points_collection)
            final_points = []

            idxs = dataset['collections'][selected_collection_key]['labels'][sensor]['idxs']
            idxs_limit_points = dataset['collections'][selected_collection_key]['labels'][sensor]['idxs_limit_points']
            sensor_idx = list(
                dataset['collections'][selected_collection_key]['labels'].keys()).index(sensor)

            for idx, point in enumerate(gen_points):
                if idx in idxs_limit_points:
                    r, g, b = 50, 70, 20
                elif idx in idxs:
                    r, g, b = 100, 220, 20
                else:
                    r, g, b = 186, 189, 182

                point_color = [point[0], point[1],
                               point[2], idx, 0, sensor_idx]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                point_color[4] = rgb

                final_points.append(point_color)

            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1),
                      PointField('idx', 12, PointField.FLOAT32, 1),
                      PointField('rgb', 20, PointField.UINT32, 1),
                      PointField('sensor_idx', 24, PointField.FLOAT32, 1)
                      ]
            pointcloud_msg_final = pc2.create_cloud(
                pointcloud_msg.header, fields, final_points)

            # create a new point cloud2, and change a value related to the idx
            graphics['ros']['PubPointCloud'][sensor].publish(
                pointcloud_msg_final)

    # Create a new marker array which contains only the marker related to the selected collection
    # Publish the pattern data
    marker_array_1 = MarkerArray()
    for marker in graphics['ros']['MarkersPattern'].markers:
        prefix = marker.header.frame_id[:3]
        if prefix == 'c' + str(selected_collection_key) + '_':
            marker_array_1.markers.append(marker)
            marker_array_1.markers[-1].action = Marker.ADD
        elif not previous_selected_collection_key == selected_collection_key and prefix == 'c' + str(
                previous_selected_collection_key) + '_':
            marker_array_1.markers.append(marker)
            marker_array_1.markers[-1].action = Marker.DELETE
    graphics['ros']['PubPattern'].publish(marker_array_1)

    # Create a new marker array which contains only the marker related to the selected collection
    # Publish the robot_mesh_
    marker_array_2 = MarkerArray()
    for marker in graphics['ros']['robot_mesh_markers'].markers:
        prefix = marker.header.frame_id[:3]
        if prefix == 'c' + str(selected_collection_key) + '_':
            marker_array_2.markers.append(marker)
            marker_array_2.markers[-1].action = Marker.ADD
        elif not previous_selected_collection_key == selected_collection_key and prefix == 'c' + str(
                previous_selected_collection_key) + '_':
            marker_array_2.markers.append(marker)
            marker_array_2.markers[-1].action = Marker.DELETE
    graphics['ros']['publisher_models'].publish(
        graphics['ros']['robot_mesh_markers'])

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
                        drawSquare2D(gui_image, point['x'], point['y'], size=5, color=(50, 190, 0))

                    # Draw a line segment for each pair of consecutive points
                    for point_start, point_end in zip(clicked_sensor_points[:-1], clicked_sensor_points[1:]):
                        cv2.line(gui_image, pt1=(point_start['x'], point_start['y']),
                                 pt2=(point_end['x'], point_end['y']),
                                 color=(0, 0, 255), thickness=1)

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
