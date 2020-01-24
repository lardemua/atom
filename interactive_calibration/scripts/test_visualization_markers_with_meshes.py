#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------

import rospy
import urdf_parser_py
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import Header, ColorRGBA
from urdf_parser_py.urdf import URDF

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------

# ------------------------
#      FUNCTIONS         #
# ------------------------
from visualization_msgs.msg import MarkerArray, Marker

if __name__ == "__main__":
    # Initialize ROS stuff
    rospy.init_node("test_visualization_markers_with_meshes")
    rospy.sleep(0.1)

    # xacro_file = rospy.get_param('~xacro_file')
    # rospy.loginfo('xacro_file is ' + xacro_file)
    # prefix = rospy.get_param('~prefix')
    # rospy.loginfo('prefix is ' + prefix)

    markers = MarkerArray()
    pub_markers = rospy.Publisher('test_meshes', MarkerArray, queue_size=0, latch=True)

    # Parse robot description from param /robot_description
    rospy.loginfo('Reading xml xacro file ...')
    # xml_robot = URDF.from_xml_file(xacro_file)
    xml_robot = URDF.from_parameter_server()

    # print("xml_robot " + str(xml_robot))
    rospy.loginfo('Changing link names ...')
    for link in xml_robot.links:
        print("Analysing link: " + str(link.name))

        if link.visuals:
            if link.visuals[0].geometry:

                if isinstance(link.visuals[0].geometry, urdf_parser_py.urdf.Mesh):
                    filename = link.visuals[0].geometry.filename

                    print("visuals:\n " + str(link.visuals[0]))

                    print("origin:\n " + str(link.visuals[0].origin))
                    x = link.visuals[0].origin.xyz[0]
                    y = link.visuals[0].origin.xyz[1]
                    z = link.visuals[0].origin.xyz[2]
                    print("mesh file: " + filename)

                    m = Marker(header=Header(frame_id=link.name, stamp=rospy.Time.now()),
                               ns=link.name, id=0, frame_locked=True,
                               type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(0),
                               pose=Pose(position=Point(x=x, y=y, z=z),
                                         orientation=Quaternion(x=0, y=0, z=0, w=1)),
                               scale=Vector3(x=1.0, y=1.0, z=1.0),
                               color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.1)
                               )
                    m.mesh_resource = filename
                    m.mesh_use_embedded_materials = True
                    markers.markers.append(m)

                elif isinstance(link.visuals[0].geometry, urdf_parser_py.urdf.Box):
                    sx = link.visuals[0].geometry.size[0]
                    sy = link.visuals[0].geometry.size[1]
                    sz = link.visuals[0].geometry.size[2]

                    x = link.visuals[0].origin.xyz[0]
                    y = link.visuals[0].origin.xyz[1]
                    z = link.visuals[0].origin.xyz[2]

                    m = Marker(header=Header(frame_id=link.name, stamp=rospy.Time.now()),
                               ns=link.name, id=0, frame_locked=True,
                               type=Marker.CUBE, action=Marker.ADD, lifetime=rospy.Duration(0),
                               pose=Pose(position=Point(x=x, y=y, z=z),
                                         orientation=Quaternion(x=0, y=0, z=0, w=1)),
                               scale=Vector3(x=sx, y=sy, z=sz),
                               color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.2)
                               )
                    markers.markers.append(m)

                else:
                    print('Unknown link type' + str(type(link.visuals[0].geometry)))
                    print("visuals:\n " + str(link.visuals[0]))

    while not rospy.is_shutdown():
        pub_markers.publish(markers)
        rospy.sleep(1)
