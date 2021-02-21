#!/usr/bin/env python3

import pypcd
import rospy
from sensor_msgs.msg import PointCloud2

pc = pypcd.PointCloud.from_path('/home/mike/datasets/mmtbot/dataset_21_02_2021/3dlidar_0.pcd')

pc.save_pcd('/home/mike/Desktop/pointcloud.pcd')
# pc = pypcd.PointCloud.from_path('/home/mike/datasets/agrob/agrob_paper/train_dataset/vlp16_20.pcd')



# def cb(msg):
#     pc = pypcd.PointCloud.from_msg(msg)
#     pc.save('foo.pcd', compression='binary_compressed')
#     # maybe manipulate your pointcloud
#     pc.pc_data['x'] *= -1
#     outmsg = pc.to_msg()
#     # you'll probably need to set the header
#     outmsg.header = msg.header
#     pub.publish(outmsg)
#
# # ...
#
# pypcd.

# sub = rospy.Subscriber('incloud', PointCloud2)
# pub = rospy.Publisher('outcloud', PointCloud2, cb)
# rospy.init('pypcd_node')
# rospy.spin()
