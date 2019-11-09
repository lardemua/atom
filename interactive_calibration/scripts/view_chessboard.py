#!/usr/bin/env python

import argparse
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SimpleChessboardDetector:

    def __init__(self, topic, num_x, num_y):
        self.sub    = rospy.Subscriber(topic, Image, self.onImage, queue_size=1)
        self.bridge = CvBridge()
        self.size = (num_x, num_y)

    def onImage(self, image_msg):
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(gray, self.size)
        cv2.drawChessboardCorners(image, self.size, corners, found)  # Draw and display the corners

        cv2.imshow('show', image)
        key = cv2.waitKey(1)
        if key & 0xff == ord('q'):
            rospy.signal_shutdown(1)

def main():
    rospy.init_node('detect_chessboard')

    parser = argparse.ArgumentParser()
    parser.add_argument("topic", help="Topic name to subscribe.", metavar='topic', type=str)
    parser.add_argument("-x", "--chess_num_x", help="Chessboard's number of corners in horizontal dimension.",
                    type=int, required=True)
    parser.add_argument("-y", "--chess_num_y", help="Chessboard's number of corners in vertical dimension.",
                    type=int, required=True)
    args = vars(parser.parse_args())

    scd = SimpleChessboardDetector(args['topic'], args['chess_num_x'], args['chess_num_y'])
    rospy.spin()

if __name__ == '__main__': main()

