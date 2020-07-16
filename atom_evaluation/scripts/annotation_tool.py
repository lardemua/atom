#!/usr/bin/env python

"""
Image annotation tool
"""

import cv2
import numpy as np

drawing = False  # true if mouse is pressed
mode = True  # if True, draw rectangle. Press 'm' to toggle to curve
ix, iy = -1, -1


# mouse callback function
def draw_line(event, x, y, flags, param):
    global ix, iy, drawing, mode

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    # elif event == cv2.EVENT_MOUSEMOVE:
        # if drawing == True:
        #     cv2.line(img, (ix, iy), (x, y), (0, 255, 0), 2)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.line(img, (ix, iy), (x, y), (0, 255, 0), 2)


if __name__ == "__main__":
    img = np.zeros((512, 512, 3), np.uint8)
    cv2.namedWindow('image')
    cv2.setMouseCallback('image', draw_line)

    while (1):
        cv2.imshow('image', img)
        k = cv2.waitKey(1) & 0xFF

    cv2.destroyAllWindows()
