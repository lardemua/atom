#!/usr/bin/python3
import cv2
import numpy as np

def detect_pattern(img, equalize_histogram=False):
        if len(img.shape) == 3:  # convert to gray if it is an rgb image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        if equalize_histogram:
            gray = cv2.equalizeHist(gray)

        charuco_dict = {'DICT_5X5_100': cv2.aruco.DICT_5X5_100}
        cdictionary = charuco_dict['DICT_5X5_100']
        size = (11, 8)
        number_of_corners = size[0]*size[1]
        dictionary = cv2.aruco.getPredefinedDictionary(cdictionary)
        board = cv2.aruco.CharucoBoard_create(
            size[0] + 1, size[1] + 1, 0.06, 0.045, dictionary)

        # more information here https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html:w
        params = cv2.aruco.DetectorParameters_create()

        # setup initial data
        params.adaptiveThreshConstant = 2
        # params.adaptiveThreshWinSizeMin = 3
        # params.adaptiveThreshWinSizeMax = 10
        # params.adaptiveThreshWinSizeStep = 5
        params.minMarkerPerimeterRate = 0.003
        params.maxMarkerPerimeterRate = 4
        params.minCornerDistanceRate = 0.1
        params.markerBorderBits = 1
        params.minOtsuStdDev = 15
        params.perspectiveRemoveIgnoredMarginPerCell = .1
        params.maxErroneousBitsInBorderRate = .15
        params.errorCorrectionRate = .6

        # param.doCornerRefinement = False
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=params)
        # print('corners = ' + str(corners))
        # print(len(ids)) # this value is greater with more detections on the pattern
        corners, ids, rejected, _ = cv2.aruco.refineDetectedMarkers(
            gray, board, corners, ids, rejected)

        if len(corners) > 4:
            ret, ccorners, cids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board)
            criteria = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 500, 0.0001)

            # A valid detection must have at least 25% of the total number of corners.
            detected = ccorners is not None and len(
                ccorners) > number_of_corners / 4
            if detected:
                return {'detected': detected, 'keypoints': ccorners, 'ids': cids.ravel().tolist()}

        return {"detected": False, "keypoints": np.array([]), "ids": []}

def draw_keypoints(img, result):
    if result['keypoints'] is None or len(result['keypoints']) == 0:
        detected_points = 0
        # print(points)
        return detected_points
    points = result['keypoints'].astype(np.int32)
    # print(len(points))
    detected_points = (len(points))
    for point in points:
        cv2.drawMarker(img, tuple(point[0]),
                       (0, 0, 255), cv2.MARKER_CROSS, 14)
        cv2.circle(img, tuple(point[0]), 7,
                   (0, 255, 0), lineType=cv2.LINE_AA)
    return detected_points