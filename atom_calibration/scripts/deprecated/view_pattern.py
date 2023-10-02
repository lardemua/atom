#!/usr/bin/env python3

import sys
import argparse
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from atom_calibration.collect import patterns


class SimplePatternDetector:

    def __init__(self, options):
        size = {"x": options['num_x'], "y": options['num_y']}
        length = options['length']
        inner_length = options['inner_length']
        dictionary = options['dict']
        self.options = options

        if options['type'] == 'charuco':
            self.pattern = patterns.CharucoPattern(size, length, inner_length, dictionary)
        elif options['type'] == 'chessboard':
            self.pattern = patterns.ChessboardPattern(size, length)
        else:
            rospy.logerr("Unknown pattern '{}'".format(options['type']))
            sys.exit(1)

        self.sub = rospy.Subscriber(options['topic'], Image, self.onImageReceived, queue_size=1)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(options['topic'] + '/labeled', Image, queue_size=1)

    def onImageReceived(self, image_msg):

        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        result = self.pattern.detect(image, equalize_histogram=False)




        self.pattern.drawKeypoints(image, result)

        image_msg_out = self.bridge.cv2_to_imgmsg(image, 'passthrough')
        self.image_pub.publish(image_msg_out)

        cv2.namedWindow(self.options['topic'], cv2.WINDOW_NORMAL)
        cv2.imshow(self.options['topic'], image)
        key = cv2.waitKey(1)
        if key & 0xff == ord('q'):
            rospy.signal_shutdown(1)


def main():
    rospy.init_node('detect_chessboard', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("topic", help="Topic name to subscribe.", metavar='topic', type=str)
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

    scd = SimplePatternDetector(args)
    rospy.spin()


if __name__ == '__main__':
    main()
