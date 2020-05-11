import cv2
import numpy as np


class ChessboardPattern(object):
    def __init__(self, size, length):
        self.size = (size["x"], size["y"])
        self.length = length

    def detect(self, image, equalize_histogram=False):

        if len(image.shape) == 3:  # convert to gray if it is an rgb image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if equalize_histogram:
            gray = cv2.equalizeHist(gray)

        # Find chessboard corners
        found, corners = cv2.findChessboardCorners(gray, self.size)
        if not found:
            return {"detected": False, 'keypoints': corners, 'ids': []}

        # WARNING: this is a quick hack to maintain the chessboard corners
        # in the right place.
        diff = corners[0][0][0] - corners[self.size[0] - 1][0][0]
        if diff > 0:
            corners = np.array(np.flipud(corners))

        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # spcorners = cv2.cornerSubPix(gray, corners, self.size, (-1, -1), criteria)
        spcorners = corners

        return {"detected": True, 'keypoints': spcorners, 'ids': range(0, len(spcorners))}

    def drawKeypoints(self, image, result):
        if result['keypoints'] is None or len(result['keypoints']) == 0:
            return

        for point in result['keypoints']:
            cv2.drawMarker(image, tuple(point[0]), (0, 0, 255), cv2.MARKER_CROSS, 14)
            cv2.circle(image, tuple(point[0]), 7, (0, 255, 0), lineType=cv2.LINE_AA)


class CharucoPattern(object):
    def __init__(self, size, length, marker_length, dictionary='DICT_5X5_100'):

        # string to charuco dictionary conversion
        charuco_dict = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000
        }

        if dictionary in charuco_dict:
            cdictionary = charuco_dict[dictionary]
        else:
            print('Invalid dictionary set on json configuration file. Using the default DICT_5X5_100.')
            cdictionary = charuco_dict['DICT_5X5_100']

        self.size = (size["x"], size["y"])
        self.number_of_corners = size["x"] * size["y"]
        self.dictionary = cv2.aruco.getPredefinedDictionary(cdictionary)
        self.board = cv2.aruco.CharucoBoard_create(size["x"] + 1, size["y"] + 1, length, marker_length, self.dictionary)

    def detect(self, image, equalize_histogram=False):

        if len(image.shape) == 3:  # convert to gray if it is an rgb image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        if equalize_histogram:  # equalize image histogram
            gray = cv2.equalizeHist(gray)

        params = cv2.aruco.DetectorParameters_create()

        # setup initial data
        # params.adaptiveThreshConstant = 2
        params.adaptiveThreshWinSizeMin = 41
        params.adaptiveThreshWinSizeMax = 81
        # params.adaptiveThreshWinSizeStep = 5
        # params.minMarkerPerimeterRate = 0.003
        # params.maxMarkerPerimeterRate = 4
        # params.minCornerDistanceRate = 0.1
        # params.markerBorderBits = 1
        # params.minOtsuStdDev = 15
        # params.perspectiveRemoveIgnoredMarginPerCell = .1
        # params.maxErroneousBitsInBorderRate = .15
        # params.errorCorrectionRate = .6

        # param.doCornerRefinement = False
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=params)
        corners, ids, rejected, _ = cv2.aruco.refineDetectedMarkers(gray, self.board, corners, ids, rejected)

        print('Detected ' + str(len(corners)) + ' corners')
        if len(corners) > 4:
            ret, ccorners, cids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)

            # A valid detection must have at least half the total number of corners.
            detected = ccorners is not None and len(ccorners) > self.number_of_corners / 4
            if detected:
                return {'detected': detected, 'keypoints': ccorners, 'ids': cids.ravel().tolist()}

        return {"detected": False, 'keypoints': np.array([]), 'ids': []}

    def drawKeypoints(self, image, result):
        if result['keypoints'] is None or len(result['keypoints']) == 0:
            return

        for point in result['keypoints']:
            cv2.drawMarker(image, tuple(point[0]), (0, 0, 255), cv2.MARKER_CROSS, 14)
            cv2.circle(image, tuple(point[0]), 7, (0, 255, 0), lineType=cv2.LINE_AA)
