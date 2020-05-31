#!/usr/bin/env python

# -------------------------------------------------------------------------------
# --- IMPORTS (standard, then third party, then my own modules)
# -------------------------------------------------------------------------------
import rospy
import math
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

import tf.transformations as trf


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def main():
    # Initialize ROS stuff
    rospy.init_node("optimization_node")
    pub = rospy.Publisher('robot_meshes', MarkerArray, queue_size=0, latch=True)

    # Draw the chessboard
    markers = MarkerArray()
    m = Marker(header=Header(frame_id='base_link', stamp=rospy.Time.now()),
               ns='charuco', id=0, frame_locked=True,
               type=Marker.MESH_RESOURCE, action=Marker.ADD, lifetime=rospy.Duration(0),
               pose=Pose(position=Point(x=-0.9, y=-0.176*2, z=0.5),
                         orientation=Quaternion(*trf.quaternion_from_euler(math.pi*0.5,0,math.atan2(-0.176,-1))-math.pi*0.1)),
               scale=Vector3(x=1.5, y=1.5, z=1.5),
               color=ColorRGBA(r=.5, g=.5, b=.5, a=1))
    m.mesh_resource = 'package://ur_e_description/meshes/objects/charuco_5x5.dae'
    m.mesh_use_embedded_materials = True
    markers.markers.append(m)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(markers)
        rate.sleep()


if __name__ == '__main__':
    main()
