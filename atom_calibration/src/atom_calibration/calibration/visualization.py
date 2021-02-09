#!/usr/bin/env python
"""
Reads a set of data and labels from a group of sensors in a json file and calibrates the poses of these sensors.
"""

# stdlib
import copy
import math
import os
import pprint

# 3rd-party
import cv2
import ros_numpy
# import numpy as np  # TODO Eurico, line  fails if I don't do this
import rospy
import numpy as np
import tf
from atom_core.cache import Cache

from rospy_message_converter import message_converter
from atom_core.rospy_urdf_to_rviz_converter import urdfToMarkerArray
from std_msgs.msg import Header, ColorRGBA
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import Image, sensor_msgs, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from cv_bridge import CvBridge

from matplotlib import cm
from OptimizationUtils import utilities as opt_utilities

# own packages
from atom_core.naming import generateName
from atom_core.config_io import readXacroFile, execute, uriReader
from atom_core.dataset_io import getCvImageFromDictionary, getPointCloudMessageFromDictionary, genCollectionPrefix


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------

@Cache(args_to_ignore=['dataset'])
def getCvImageFromCollectionSensor(collection_key, sensor_key, dataset):
    dictionary = dataset['collections'][collection_key]['data'][sensor_key]
    return getCvImageFromDictionary(dictionary)


def createPatternMarkers(frame_id, ns, collection_key, now, dataset, graphics):
    markers = MarkerArray()

    # Draw pattern frame lines_sampled (top, left, right, bottom)
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-frame_sampled', id=0, frame_locked=True,
                    type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.025, y=0.025, z=0.025),
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
                    type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
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
    marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                    ns=ns + '-transitions', id=0, frame_locked=True,
                    type=Marker.POINTS, action=Marker.ADD, lifetime=rospy.Duration(0),
                    pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                    scale=Vector3(x=0.015, y=0.015, z=0),
                    color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                    g=graphics['collections'][collection_key]['color'][1],
                                    b=graphics['collections'][collection_key]['color'][2], a=1.0))

    pts = dataset['patterns']['transitions']['vertical']
    pts.extend(dataset['patterns']['transitions']['horizontal'])
    for pt in pts:
        marker.points.append(Point(x=pt['x'], y=pt['y'], z=0))

    markers.markers.append(marker)

    # Draw the mesh, if one is provided
    if not dataset['calibration_config']['calibration_pattern']['mesh_file'] == "":
        # rgba = graphics['collections'][collection_key]['color']
        # color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=1))
        m = Marker(header=Header(frame_id=frame_id, stamp=now),
                   ns=str(collection_key) + '-mesh', id=0, frame_locked=True,
                   type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(0),
                   pose=Pose(position=Point(x=0, y=0, z=0),
                             orientation=Quaternion(x=0, y=0, z=0, w=1)),
                   scale=Vector3(x=1.0, y=1.0, z=1.0),
                   color=ColorRGBA(r=1, g=1, b=1, a=1))

        mesh_file, _, _ = uriReader(dataset['calibration_config']['calibration_pattern']['mesh_file'])
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
    graphics = {'collections': {}, 'sensors': {}, 'pattern': {}, 'ros': {}, 'args': args}

    # Initialize ROS stuff
    rospy.init_node("calibrate")
    graphics['ros']['tf_broadcaster'] = tf.TransformBroadcaster()
    graphics['ros']['publisher_models'] = rospy.Publisher('~robot_meshes', MarkerArray, queue_size=0, latch=True)
    now = rospy.Time.now()

    # Parse xacro description file
    description_file, _, _ = uriReader(dataset['calibration_config']['description_file'])
    rospy.loginfo('Reading description file ' + description_file + '...')
    xml_robot = readXacroFile(description_file)

    pattern = dataset['calibration_config']['calibration_pattern']

    # Create colormaps to be used for coloring the elements. Each collection contains a color, each sensor likewise.
    graphics['pattern']['colormap'] = cm.gist_rainbow(
        np.linspace(0, 1, pattern['dimension']['x'] * pattern['dimension']['y']))

    graphics['collections']['colormap'] = cm.tab20b(np.linspace(0, 1, len(dataset['collections'].keys())))
    for idx, collection_key in enumerate(sorted(dataset['collections'].keys())):
        graphics['collections'][str(collection_key)] = {'color': graphics['collections']['colormap'][idx, :]}

    # color_map_sensors = cm.gist_rainbow(np.linspace(0, 1, len(dataset['sensors'].keys())))
    # for idx, sensor_key in enumerate(sorted(dataset['sensors'].keys())):
    #     dataset['sensors'][str(sensor_key)]['color'] = color_map_sensors[idx, :]

    # Create image publishers ----------------------------------------------------------
    # We need to republish a new image at every visualization
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not collection['labels'][str(sensor_key)]['detected']:  # not detected by sensor in collection
                continue

            if sensor['msg_type'] == 'Image':
                msg_type = sensor_msgs.msg.Image
                topic = dataset['calibration_config']['sensors'][sensor_key]['topic_name']
                topic_name = '~c' + str(collection_key) + topic + '/labeled'
                graphics['collections'][collection_key][str(sensor_key)] = {'publisher': rospy.Publisher(
                    topic_name, msg_type, queue_size=0, latch=True)}

                msg_type = sensor_msgs.msg.CameraInfo
                topic_name = '~c' + str(collection_key) + '/' + str(sensor_key) + '/camera_info'
                graphics['collections'][collection_key][str(sensor_key)]['publisher_camera_info'] = \
                    rospy.Publisher(topic_name, msg_type, queue_size=0, latch=True)

    # Create Labeled Data publishers ----------------------------------------------------------
    markers = MarkerArray()
    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not collection['labels'][str(sensor_key)]['detected']:  # not detected by sensor in collection
                continue

            if sensor['msg_type'] == 'LaserScan':  # -------- Publish the laser scan data ------------------------------
                frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.POINTS, action=Marker.ADD, lifetime=rospy.Duration(0),
                                pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
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

            if sensor['msg_type'] == 'PointCloud2':  # -------- Publish the velodyne data ------------------------------

                cloud_msg = getPointCloudMessageFromDictionary(collection['data'][sensor_key])

                # Get LiDAR points that belong to the pattern
                idxs = collection['labels'][sensor_key]['idxs']
                pc = ros_numpy.numpify(cloud_msg)
                points = np.zeros((pc.shape[0], 3))
                points[:, 0] = pc['x']
                points[:, 1] = pc['y']
                points[:, 2] = pc['z']

                frame_id = genCollectionPrefix(collection_key, collection['data'][sensor_key]['header']['frame_id'])
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key), id=0, frame_locked=True,
                                type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                pose=Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                scale=Vector3(x=0.02, y=0.02, z=0.02),
                                color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                g=graphics['collections'][collection_key]['color'][1],
                                                b=graphics['collections'][collection_key]['color'][2], a=0.4)
                                )

                for idx in idxs:
                    marker.points.append(Point(x=points[idx, 0], y=points[idx, 1], z=points[idx, 2]))

                markers.markers.append(copy.deepcopy(marker))

                # Visualize LiDAR corner points
                marker = Marker(header=Header(frame_id=frame_id, stamp=now),
                                ns=str(collection_key) + '-' + str(sensor_key) + '-limit_points', id=0,
                                frame_locked=True,
                                type=Marker.SPHERE_LIST, action=Marker.ADD, lifetime=rospy.Duration(0),
                                pose=Pose(position=Point(x=0, y=0, z=0),
                                          orientation=Quaternion(x=0, y=0, z=0, w=1)),
                                scale=Vector3(x=0.07, y=0.07, z=0.07),
                                color=ColorRGBA(r=graphics['collections'][collection_key]['color'][0],
                                                g=graphics['collections'][collection_key]['color'][1],
                                                b=graphics['collections'][collection_key]['color'][2], a=0.8)
                                )

                for pt in collection['labels'][sensor_key]['limit_points']:
                    marker.points.append(Point(x=pt['x'], y=pt['y'], z=pt['z']))

                markers.markers.append(copy.deepcopy(marker))

    graphics['ros']['MarkersLabeled'] = markers
    graphics['ros']['PubLabeled'] = rospy.Publisher('~labeled_data', MarkerArray, queue_size=0, latch=True)

    # -----------------------------------------------------------------------------------------------------
    # -------- Robot meshes
    # -----------------------------------------------------------------------------------------------------
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
                              namespace=selected_collection_key,
                              rgba=rgba)
        markers.markers.extend(m.markers)
    else:  # render robot meshes for all collections
        for collection_key, collection in dataset['collections'].items():
            # rgba = graphics['collections'][collection_key]['color']
            # rgba[3] = 0.4  # change the alpha
            rgba = [.5, .5, .5, 0.7]  # best color we could find
            m = urdfToMarkerArray(xml_robot, frame_id_prefix=genCollectionPrefix(collection_key, ''),
                                  namespace=collection_key,
                                  rgba=rgba)
            markers.markers.extend(m.markers)

    graphics['ros']['robot_mesh_markers'] = markers

    # -----------------------------------------------------------------------------------------------------
    # -------- Publish the pattern data
    # -----------------------------------------------------------------------------------------------------
    if dataset['calibration_config']['calibration_pattern']['fixed']:  # Draw single pattern for selected collection key
        frame_id = generateName(dataset['calibration_config']['calibration_pattern']['link'],
                                prefix='c' + selected_collection_key)
        ns = str(selected_collection_key)
        markers = createPatternMarkers(frame_id, ns, selected_collection_key, now, dataset, graphics)
    else:  # Draw a pattern per collection
        markers = MarkerArray()
        for idx, (collection_key, collection) in enumerate(dataset['collections'].items()):
            frame_id = generateName(dataset['calibration_config']['calibration_pattern']['link'],
                                    prefix='c' + collection_key)
            ns = str(collection_key)
            collection_markers = createPatternMarkers(frame_id, ns, collection_key, now, dataset, graphics)
            markers.markers.extend(collection_markers.markers)

    graphics['ros']['MarkersPattern'] = markers
    graphics['ros']['PubPattern'] = rospy.Publisher('~patterns', MarkerArray, queue_size=0, latch=True)

    # Create LaserBeams Publisher -----------------------------------------------------------
    # This one is recomputed every time in the objective function, so just create the generic properties.
    markers = MarkerArray()

    for collection_key, collection in dataset['collections'].items():
        for sensor_key, sensor in dataset['sensors'].items():
            if not collection['labels'][sensor_key]['detected']:  # chess not detected by sensor in collection
                continue
            if sensor['msg_type'] == 'LaserScan' or sensor['msg_type'] == 'PointCloud2':
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

    return graphics


def visualizationFunction(models):
    # Get the data from the meshes
    dataset = models['dataset']
    args = models['args']
    collections = models['dataset']['collections']
    sensors = models['dataset']['sensors']
    patterns = models['dataset']['patterns']
    config = models['dataset']['calibration_config']
    graphics = models['graphics']

    now = rospy.Time.now()  # time used to publish all visualization messages

    for collection_key, collection in collections.items():

        # To have a fully connected tree, must connect the instances of the tf tree of every collection into a single
        # tree. We do this by publishing an identity transform between the configured world link and hte world link
        # of each collection.
        parent = config['world_link']
        child = generateName(config['world_link'], prefix='c' + collection_key)
        graphics['ros']['tf_broadcaster'].sendTransform((0, 0, 0), (0, 0, 0, 1), now, child, parent)

        # Publish all current transforms
        for transform_key, transform in collection['transforms'].items():
            # TODO after https://github.com/lardemua/AtlasCarCalibration/issues/54 this will be unnecessary
            parent = 'c' + collection_key + '_' + transform_key.split('-')[0]
            child = 'c' + collection_key + '_' + transform_key.split('-')[1]
            graphics['ros']['tf_broadcaster'].sendTransform(transform['trans'], transform['quat'], now, child, parent)

        # TODO Andre, remove this When you are finished. Just a hack for being able to visualize the wheels
        parent = 'c' + collection_key + '_' + 'base_link'
        child = 'c' + collection_key + '_' + 'front_left_wheel_link'
        graphics['ros']['tf_broadcaster'].sendTransform([0.256, 0.285, 0.033], [0, 0, 0, 1], now, child, parent)

        child = 'c' + collection_key + '_' + 'front_right_wheel_link'
        graphics['ros']['tf_broadcaster'].sendTransform([0.256, -0.285, 0.033], [0, 0, 0, 1], now, child, parent)

        child = 'c' + collection_key + '_' + 'rear_left_wheel_link'
        graphics['ros']['tf_broadcaster'].sendTransform([-0.256, 0.285, 0.033], [0, 0, 0, 1], now, child, parent)

        child = 'c' + collection_key + '_' + 'rear_right_wheel_link'
        graphics['ros']['tf_broadcaster'].sendTransform([-0.256, -0.285, 0.033], [0, 0, 0, 1], now, child, parent)
        # Remove until this point

    # Update markers stamp, so that rviz uses newer transforms to compute their poses.
    for marker in graphics['ros']['robot_mesh_markers'].markers:
        marker.header.stamp = now

    # Publish the meshes
    graphics['ros']['publisher_models'].publish(graphics['ros']['robot_mesh_markers'])

    # Not needed now that pattern transforms are added to the collection['transforms']
    # Publishes the chessboards transforms
    # for idx, (collection_pattern_key, collection_pattern) in enumerate(patterns['collections'].items()):
    #     parent = 'base_link'
    #     child = 'c' + collection_pattern_key + '_pattern_link'
    #     graphics['ros']['tf_broadcaster'].sendTransform(collection_pattern['trans'], collection_pattern['quat'],
    #                                                     now, child, parent)

    # Publish patterns
    for marker in graphics['ros']['MarkersPattern'].markers:
        marker.header.stamp = now
    graphics['ros']['PubPattern'].publish(graphics['ros']['MarkersPattern'])

    # Publish Labelled Data
    for marker in graphics['ros']['MarkersLabeled'].markers:
        marker.header.stamp = now
    graphics['ros']['PubLabeled'].publish(graphics['ros']['MarkersLabeled'])

    # Publish Laser Beams
    for marker in graphics['ros']['MarkersLaserBeams'].markers:
        marker.header.stamp = now
    graphics['ros']['PubLaserBeams'].publish(graphics['ros']['MarkersLaserBeams'])

    # Publish Annotated images
    for collection_key, collection in collections.items():
        for sensor_key, sensor in sensors.items():

            if not collection['labels'][sensor_key]['detected']:  # not detected by sensor in collection
                continue

            if sensor['msg_type'] == 'Image':
                if args['show_images']:
                    image = copy.deepcopy(getCvImageFromCollectionSensor(collection_key, sensor_key, dataset))
                    width = collection['data'][sensor_key]['width']
                    height = collection['data'][sensor_key]['height']
                    diagonal = math.sqrt(width ** 2 + height ** 2)
                    cm = graphics['pattern']['colormap']

                    # Draw projected points (as dots)
                    for idx, point in enumerate(collection['labels'][sensor_key]['idxs_projected']):
                        x = int(round(point['x']))
                        y = int(round(point['y']))
                        color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                        cv2.line(image, (x, y), (x, y), color, int(6E-3 * diagonal))

                    # Draw ground truth points (as squares)
                    for idx, point in enumerate(collection['labels'][sensor_key]['idxs']):
                        x = int(round(point['x']))
                        y = int(round(point['y']))
                        color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                        opt_utilities.drawSquare2D(image, x, y, int(8E-3 * diagonal), color=color, thickness=1)

                    # Draw initial projected points (as crosses)
                    for idx, point in enumerate(collection['labels'][sensor_key]['idxs_initial']):
                        x = int(round(point['x']))
                        y = int(round(point['y']))
                        color = (cm[idx, 2] * 255, cm[idx, 1] * 255, cm[idx, 0] * 255)
                        opt_utilities.drawCross2D(image, x, y, int(8E-3 * diagonal), color=color, thickness=1)

                    msg = CvBridge().cv2_to_imgmsg(image, "bgr8")

                    msg.header.frame_id = 'c' + collection_key + '_' + sensor['parent']
                    graphics['collections'][collection_key][sensor_key]['publisher'].publish(msg)

                    # Publish camera info message
                    camera_info_msg = message_converter.convert_dictionary_to_ros_message('sensor_msgs/CameraInfo',
                                                                                          sensor['camera_info'])
                    camera_info_msg.header.frame_id = msg.header.frame_id
                    graphics['collections'][collection_key][sensor_key]['publisher_camera_info'].publish(
                        camera_info_msg)

            elif sensor['msg_type'] == 'LaserScan':
                pass
            elif sensor['msg_type'] == 'PointCloud2':
                pass
            else:
                raise ValueError("Unknown sensor msg_type")
