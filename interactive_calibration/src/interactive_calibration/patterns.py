
import cv2
import numpy as np


class Pattern:
    def __init__(self):
        pass

    def detect(self, image):
        return {"detected": False, 'keypoints': np.array([])}

    def drawKeypoints(self, image, data):
        pass


class ChessboardPattern(Pattern):
    def __init__(self, size, length):
        self.size = size
        self.length = length

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        found, corners = cv2.findChessboardCorners(gray, self.size)
        if not found:
            return {"detected": False, 'keypoints': corners}

        # WARNING: this is a quick hack to maintain the chessboard corners
        # in the right place.
        diff = corners[0][0][0] - corners[self.size[0]-1][0][0]
        if diff > 0:
            corners = np.array(np.flipud(corners))

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        spcorners = cv2.cornerSubPix(gray, corners, self.size, (-1, -1), criteria)

        return {"detected": True, 'keypoints': spcorners}

    def drawKeypoints(self, image, result):
        if result['keypoints'] is None or len(result['keypoints']) == 0:
            return

        cv2.drawChessboardCorners(image, self.size, result['keypoints'], result['detected'])


class CharucoPattern(Pattern):
    def __init__(self, size, length, marker_length):

        self.size = size
        self.number_of_corners = (size[0]-1) * (size[1]-1)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard_create(size[0], size[1], length, marker_length, self.dictionary)

    def detect(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
        corners, ids, rejected, recoverd = cv2.aruco.refineDetectedMarkers(gray, self.board, corners, ids, rejected)

        if len(corners) > 4:
            ret, ccorners, cids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)

            # For now, a valid detection must contain all corners
            return {'detected': len(ccorners) == self.number_of_corners,
                    'corners': ccorners, 'ids': cids}

        return {"detected": False, 'corners': np.array([]), 'ids': ids}

    def drawKeypoints(self, image, result):
        if len(result['corners']) == 0:
            return

        cv2.drawChessboardCorners(image, (self.size[0]-1, self.size[1]-1), result['corners'], result['detected'])
