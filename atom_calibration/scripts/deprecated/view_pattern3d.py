#!/usr/bin/env python3

import os
import sys
import argparse

import cv2
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from atom_calibration.collect import patterns
from atom_core.geometry import traslationRodriguesToTransform
from tf import transformations


class SimplePatternDetector:

    def __init__(self, args):
        size = {"x": args['num_x'], "y": args['num_y']}
        length = args['length']
        inner_length = args['inner_length']
        dictionary = args['dict']
        self.args = args

        if args['type'] == 'charuco':
            self.pattern = patterns.CharucoPattern(size, length, inner_length, dictionary)
        elif args['type'] == 'chessboard':
            self.pattern = patterns.ChessboardPattern(size, length)
        else:
            rospy.logerr("Unknown pattern '{}'".format(args['type']))
            sys.exit(1)


        # Get a camera_info message
        if args['camera_info_topic'] is not None:
            print('Waiting for camera_info message on topic ' + args['camera_info_topic'] + ' ...')
            self.camera_info_msg = rospy.wait_for_message(args['camera_info_topic'], CameraInfo)
            print('... received!')
            self.broadcaster = tf.TransformBroadcaster()
        else:
            self.camera_info_msg = None
            self.broadcaster = None

        self.sub = rospy.Subscriber(args['topic_name'], Image, self.onImageReceived, queue_size=1)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(args['topic_name'] + '/labeled', Image, queue_size=1)


    def onImageReceived(self, image_msg):

        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        nx = self.args['num_x']
        ny = self.args['num_y']
        square = self.args['length']
        K = np.ndarray((3, 3), dtype=float, buffer=np.array(self.camera_info_msg.K))
        D = np.ndarray((5, 1), dtype=float, buffer=np.array(self.camera_info_msg.D))
 

        print('Line 50')
        result = self.pattern.detect(image, equalize_histogram=False)
        # print(result)

        print('Line 50')
        self.pattern.drawKeypoints(image, result)




        objp = np.zeros((nx * ny, 3), np.float32)
        objp[:, :2] = square * np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        # Build a numpy array with the chessboard corners
        corners = np.zeros((len(result['keypoints']), 1, 2), dtype=float)
        ids = list(range(0, len(result['keypoints'])))
        print('corners.shape' + str(corners.shape))

        points = result['keypoints'].astype(np.int32)
        for idx, (point, id) in enumerate(zip(result['keypoints'], result['ids'])):
            corners[idx, 0, 0] = point[0][0]
            corners[idx, 0, 1] = point[0][1]
            ids[idx] = id

        # Find pose of the camera w.r.t the chessboard
        filtered_objp = objp[ids]
        print('objp.shape' + str(objp.shape))
        print('filtered_objp.shape' + str(filtered_objp.shape))

        ret, rvecs, tvecs = cv2.solvePnP(objp[ids], np.array(corners, dtype=np.float32), K, D)

        print(ids)
        np_ids = np.array(ids, dtype=int)
        print(np_ids)

        # alternative using estimatePoseBoard
        print('Before')
        print('rvecs = ' + str(rvecs))
        print('tvecs = ' + str(tvecs))
        # tvecs = tvecs*2
        # rvecs = rvecs*2
        ret, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(np.array(corners, dtype=np.float32),
                                                                np_ids,
                                                                self.pattern.board,
                                                                K,
                                                                D,
                                                                rvecs, tvecs)

        print('After')
        print('rvecs = ' + str(rvecs))
        print('tvecs = ' + str(tvecs))
#         if p_rvec is None or p_tvec is None:
#             return None
#         if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
#             return None
#         cv2.aruco.drawAxis(frame,
#                         camera_matrix,
#                         dist_coeff,
#                         p_rvec,
#                         p_tvec,
#                         0.1)
# 


        sensor_T_chessboard = traslationRodriguesToTransform(tvecs, rvecs)
        trans = list(sensor_T_chessboard[0: 3, 3])
        quat = list(transformations.quaternion_from_matrix(sensor_T_chessboard)) 

        self.broadcaster.sendTransform(trans, quat, rospy.Time.now(), 
                                       'pattern', image_msg.header.frame_id)


        
        # Try to draw frame on the image
        cv2.drawFrameAxes(image, K, D, rvecs, tvecs, 0.5)
        # https://docs.opencv.org/3.4/df/d4a/tutorial_charuco_detection.html


        image_msg_out = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.image_pub.publish(image_msg_out)

        cv2.namedWindow(self.args['topic_name'], cv2.WINDOW_NORMAL)
        cv2.imshow(self.args['topic_name'], image)
        key = cv2.waitKey(1)
        if key & 0xff == ord('q'):
            rospy.signal_shutdown(1)


def main():
    rospy.init_node('detect_chessboard', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("-tn", "--topic_name", help="Topic name to subscribe.", type=str)
    parser.add_argument("-cit", "--camera_info_topic", help="Camera info topic name to subscribe.", type=str, default=None)
    parser.add_argument("-t", "--type", help="Pattern type", type=str, choices=['charuco', 'chessboard'],
                        default='charuco')
    parser.add_argument("-d", "--dict", help="Charuco Dictionary", type=str, default='DICT_5X5_100')
    parser.add_argument("-x", "--num_x", help="Number of features in horizontal dimension.", type=int, required=True)
    parser.add_argument("-y", "--num_y", help="Number of features in vertical dimension.", type=int, required=True)
    parser.add_argument("-L", "--length", help="Lenght of the pattern marker (e.g. square, circle).", type=float,
                        required=True)
    parser.add_argument("-l", "--inner_length", help="Lenght of inner marker (e.g. aruco marker).", type=float,
                        default=0.014)
    args = vars(parser.parse_args())

    SimplePatternDetector(args)
    rospy.spin()


if __name__ == '__main__':
    main()
