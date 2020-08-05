#!/usr/bin/env python
import os

import rospy
import urdf_parser_py
from std_msgs.msg import Header, ColorRGBA
from tf import transformations
from urdf_parser_py.urdf import URDF

import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from matplotlib import cm


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def xmlDescriptionFromXacroFile(filename):
    tmp_file = '/tmp/my_xacro.xacro'  # just a temporary file
    cmd = 'xacro ' + filename + ' -o ' + tmp_file
    os.system(cmd)
    return URDF.from_xml_file(tmp_file)


def urdfToMarkerArray(xml_robot, frame_id_prefix='', namespace=None, rgba=None, verbose=False):
    """
    :param _robot_description:
    :param frame_id_prefix:
    :param namespace: if None, each link will its name. Otherwise, all links will have this namespace value value.
    :param rgba:
    :return: markers, a visualization_msgs/MarkerArray
    """

    # rospy.init_node('urdf_to_rviz_converter', anonymous=True)  # Initialize ROS node
    # rospy.loginfo('Reading xml xacro file ...')
    # xml_robot = URDF.from_parameter_server(_robot_description)  # Parse robot description from param /robot_description

    # Create colormaps to be used for coloring the elements. Each collection contains a color, each sensor likewise.
    # graphics['collections']['colormap'] = cm.tab20b(np.linspace(0, 1, len(collections.keys())))
    # for idx, collection_key in enumerate(sorted(collections.keys())):
    #     graphics['collections'][str(collection_key)] = {'color': graphics['collections']['colormap'][idx, :]}

    markers = MarkerArray()

    counter = 0
    for link in xml_robot.links:

        if verbose:
            print("Analysing link: " + str(link.name))
            print(link.name + ' has ' + str(len(link.visuals)) + ' visuals.')

        for visual in link.visuals:  # iterate through all visuals in the list
            if not visual.geometry:
                raise ValueError("Link name " + link.name + "contains visual without geometry. Are you sure your "
                                                            "urdf/xacro is correct?")
            else:
                geom = visual.geometry

            if verbose:
                print("visual: " + str(visual))
            x = y = z = 0  # origin xyz default values
            qx = qy = qz = 0  # default rotation values
            qw = 1

            if visual.origin:  # check if there is an origin
                x, y, z = visual.origin.xyz[0], visual.origin.xyz[1], visual.origin.xyz[2]
                qx, qy, qz, qw = transformations.quaternion_from_euler(visual.origin.rpy[0], visual.origin.rpy[1],
                                                                       visual.origin.rpy[2], axes='sxyz')

            if not rgba is None:  # select link color
                r, g, b, a = rgba[0], rgba[1], rgba[2], rgba[3]
            elif visual.material:
                r, g, b, a = visual.material.color.rgba
            else:
                r, g, b, a = 1, 1, 1, 1  # white by default

            # define the frame_id setting the prefix if needed
            frame_id = frame_id_prefix + link.name

            if verbose:
                print('frame id is ' + str(frame_id))

            # define the namespace
            if namespace is None:
                namespace = link.name
            else:
                namespace = namespace

            # Handle several geometries
            if isinstance(geom, urdf_parser_py.urdf.Mesh):
                if verbose:
                    print("Visual.geom of type urdf_parser_py.urdf.Mesh")

                m = Marker(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                           ns=namespace, id=counter, frame_locked=True,
                           type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(0),
                           pose=Pose(position=Point(x=x, y=y, z=z),
                                     orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)),
                           scale=Vector3(x=1.0, y=1.0, z=1.0),
                           color=ColorRGBA(r=r, g=g, b=b, a=a))
                m.mesh_resource = geom.filename
                m.mesh_use_embedded_materials = True
                markers.markers.append(m)
                counter += 1
            elif isinstance(geom, urdf_parser_py.urdf.Box):
                if verbose:
                    print("Visual.geom of type urdf_parser_py.urdf.Box")
                sx = geom.size[0]
                sy = geom.size[1]
                sz = geom.size[2]

                m = Marker(header=Header(frame_id=frame_id, stamp=rospy.Time.now()),
                           ns=namespace, id=counter, frame_locked=True,
                           type=Marker.CUBE, action=Marker.ADD, lifetime=rospy.Duration(0),
                           pose=Pose(position=Point(x=x, y=y, z=z),
                                     orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)),
                           scale=Vector3(x=sx, y=sy, z=sz),
                           color=ColorRGBA(r=r, g=g, b=b, a=a))
                markers.markers.append(m)
                counter += 1
            else:
                print("visuals:\n " + str(visual))
                raise ValueError('Unknown visual.geom type' + str(type(visual.geometry)) + " for link " +
                                 link.name)

    return markers
