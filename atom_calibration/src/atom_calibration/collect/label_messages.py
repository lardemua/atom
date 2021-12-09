import copy
import math
import random
import struct

import PIL.Image
import cv_bridge
import rospy
import scipy
import numpy as np
import atom_core.dataset_io
import ros_numpy
import functools
from scipy import ndimage

from cv_bridge import CvBridge
from cv2 import imwrite
import numpy as np
import time
import cv2


def labelImageMsg():
    pass
    # TODO the stuff from patterns.py should be here


def labelPointCloud2Msg(msg, seed_x, seed_y, seed_z, threshold, ransac_iterations,
                        ransac_threshold):
    n_inliers = 0
    labels = {}

    pc = ros_numpy.numpify(msg)
    points = np.zeros((pc.shape[0], 3))

    points[:, 0] = pc['x'].flatten()  # flatten because some pcs are of shape (npoints,1) rather than (npoints,)
    points[:, 1] = pc['y'].flatten()
    points[:, 2] = pc['z'].flatten()

    # Extract the points close to the seed point from the entire PCL
    marker_point = np.array([[seed_x, seed_y, seed_z]])
    dist = scipy.spatial.distance.cdist(marker_point, points, metric='euclidean')
    pts = points[np.transpose(dist < threshold)[:, 0], :]
    idx = np.where(np.transpose(dist < threshold)[:, 0])[0]

    # Tracker - update seed point with the average of cluster to use in the next
    # iteration
    seed_point = []
    if 0 < len(pts):
        x_sum, y_sum, z_sum = 0, 0, 0
        for i in range(0, len(pts)):
            x_sum += pts[i, 0]
            y_sum += pts[i, 1]
            z_sum += pts[i, 2]
        seed_point.append(x_sum / len(pts))
        seed_point.append(y_sum / len(pts))
        seed_point.append(z_sum / len(pts))

    # RANSAC - eliminate the tracker outliers
    number_points = pts.shape[0]
    if number_points == 0:
        labels = {'detected': False, 'idxs': [], 'idxs_limit_points': [], 'idxs_middle_points': []}
        seed_point = [seed_x, seed_y, seed_z]
        return labels, seed_point, []

    # RANSAC iterations
    for i in range(0, ransac_iterations):
        # Randomly select three points that cannot be coincident
        # nor collinear
        while True:
            idx1 = random.randint(0, number_points - 1)
            idx2 = random.randint(0, number_points - 1)
            idx3 = random.randint(0, number_points - 1)
            pt1, pt2, pt3 = pts[[idx1, idx2, idx3], :]
            # Compute the norm of position vectors
            ab = np.linalg.norm(pt2 - pt1)
            bc = np.linalg.norm(pt3 - pt2)
            ac = np.linalg.norm(pt3 - pt1)
            # Check if points are colinear
            if (ab + bc) == ac:
                continue
            # Check if are coincident
            if idx2 == idx1:
                continue
            if idx3 == idx1 or idx3 == idx2:
                continue

            # If all the conditions are satisfied, we can end the loop
            break

        # ABC Hessian coefficients and given by the external product between two vectors lying on hte plane
        Ai, Bi, Ci = np.cross(pt2 - pt1, pt3 - pt1)
        # Hessian parameter D is computed using one point that lies on the plane
        Di = - (Ai * pt1[0] + Bi * pt1[1] + Ci * pt1[2])
        # Compute the distance from all points to the plane
        # from https://www.geeksforgeeks.org/distance-between-a-point-and-a-plane-in-3-d/
        distances = abs((Ai * pts[:, 0] + Bi * pts[:, 1] + Ci * pts[:, 2] + Di)) / (
            math.sqrt(Ai * Ai + Bi * Bi + Ci * Ci))
        # Compute number of inliers for this plane hypothesis.
        # Inliers are points which have distance to the plane less than a tracker_threshold
        n_inliers_i = (distances < ransac_threshold).sum()
        # Store this as the best hypothesis if the number of inliers is larger than the previous max
        if n_inliers_i > n_inliers:
            n_inliers = n_inliers_i
            A = Ai
            B = Bi
            C = Ci
            D = Di

    # Extract the inliers
    distances = abs((A * pts[:, 0] + B * pts[:, 1] + C * pts[:, 2] + D)) / \
                (math.sqrt(A * A + B * B + C * C))
    inliers = pts[np.where(distances < ransac_threshold)]
    # Create dictionary [pcl point index, distance to plane] to select the pcl indexes of the inliers
    idx_map = dict(zip(idx, distances))
    final_idx = []
    for key in idx_map:
        if idx_map[key] < ransac_threshold:
            final_idx.append(key)

    # -------------------------------------- End of RANSAC ----------------------------------------- #

    # Update the dictionary with the labels (to be saved if the user selects the option)
    labels['detected'] = True
    labels['idxs'] = final_idx

    # ------------------------------------------------------------------------------------------------
    # -------- Extract the labelled LiDAR points on the pattern
    # ------------------------------------------------------------------------------------------------
    # STEP 1: Get labelled points into a list of dictionaries format which is suitable for later processing.
    # cloud_msg = getPointCloudMessageFromDictionary(collection['data'][sensor_key])
    # pc = ros_numpy.numpify(cloud_msg)
    # pc is computed above from the imput ros msg.

    ps = []  # list of points, each containing a dictionary with all the required information.
    for count, idx in enumerate(labels['idxs']):  # iterate all points
        x, y, z = pc['x'][idx], pc['y'][idx], pc['z'][idx]
        ps.append({'idx': idx, 'idx_in_labelled': count, 'x': x, 'y': y, 'z': z, 'limit': False})

    # STEP 2: Cluster points in pattern into groups using the theta component of the spherical coordinates.
    # We use the convention commonly used in physics, i.e:
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system

    # STEP 2.1: Convert to spherical coordinates
    for p in ps:  # iterate all points
        x, y, z = p['x'], p['y'], p['z']
        p['r'] = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        p['phi'] = math.atan2(y, x)
        p['theta'] = round(math.atan2(math.sqrt(x ** 2 + y ** 2), z), 4)  # Round to be able to cluster by
        # finding equal theta values. Other possibilities of computing theta, check link above.

    # STEP 2.2: Cluster points based on theta values
    unique_thetas = list(set([item['theta'] for item in ps]))  # Going from list of thetas to set and back
    # to list gives us the list of unique theta values.

    for p in ps:  # iterate all points and give an "idx_cluster" to each
        p['idx_cluster'] = unique_thetas.index(p['theta'])

    # STEP 3: For each cluster, get limit points, i.e. those which have min and max phi values

    # Get list of points with
    for unique_theta in unique_thetas:
        ps_in_cluster = [item for item in ps if item['theta'] == unique_theta]
        phis_in_cluster = [item['phi'] for item in ps_in_cluster]

        min_phi_idx_in_cluster = phis_in_cluster.index(min(phis_in_cluster))
        ps[ps_in_cluster[min_phi_idx_in_cluster]['idx_in_labelled']]['limit'] = True

        max_phi_idx_in_cluster = phis_in_cluster.index(max(phis_in_cluster))
        ps[ps_in_cluster[max_phi_idx_in_cluster]['idx_in_labelled']]['limit'] = True

    labels['idxs_limit_points'] = [item['idx'] for item in ps if item['limit']]

    # Count the number of limit points (just for debug)
    number_of_limit_points = len(labels['idxs_limit_points'])

    return labels, seed_point, inliers


def imageShowUInt16OrFloat32OrBool(image, window_name, max_value=5000.0):
    """
    Shows uint16 or float32 or bool images by normalizing them before using imshow
    :param image: np nd array with dtype npuint16 or floar32
    :param window_name: highgui window name
    :param max_value: value to use for white color
    """
    if not (image.dtype == np.uint16 or image.dtype == float or image.dtype == np.float32 or image.dtype == bool):
        raise ValueError('Cannot show image that is not uint16 or float. Dtype is ' + str(image.dtype))

    # Opencv can only show 8bit uint images, so we must scale
    # 0 -> 0
    # 5000 (or larger) - > 255
    image_scaled = copy.deepcopy(image)

    if image.dtype == np.uint16:
        image_scaled.astype(float)  # to not loose precision with the math
        mask = image_scaled > max_value
        image_scaled[mask] = max_value
        image_scaled = image_scaled / max_value * 255.0
        image_scaled = image_scaled.astype(np.uint8)

    elif image.dtype == float or image.dtype == np.float32:
        image_scaled = 1000.0 * image_scaled  # to go to millimeters
        mask = image_scaled > max_value
        image_scaled[mask] = max_value
        image_scaled = image_scaled / max_value * 255.0
        image_scaled = image_scaled.astype(np.uint8)

    elif image.dtype == bool:
        image_scaled = image_scaled.astype(np.uint8)
        image_scaled = image_scaled * 255

    cv2.namedWindow(window_name)
    cv2.imshow(window_name, image_scaled)


def convertDepthImage32FC1to16UC1(image_in):
    """
    The assumption is that the image_in is a float 32 bit np array, which contains the range values in meters.
    The image_out will be a 16 bit unsigned int  [0, 65536] which will store the range in millimeters.
    As a convetion, nans will be set to the maximum number available for uint16
    :param image_in: np array with dtype float32
    :return: np array with dtype uint16
    """

    # mask_nans = np.isnan(image_in)
    image_mm_float = image_in * 1000  # convert meters to millimeters
    image_mm_float = np.round(image_mm_float)  # round the millimeters
    image_out = image_mm_float.astype(np.uint16)  # side effect: nans become 0s
    return image_out


def getLinearIndexWidth(x, y, width):
    """ Gets the linear pixel index given the x,y and the image width """
    return x + y * width


def labelDepthMsg(msg, bridge=None, debug=False):
    # -------------------------------------
    # Step 1: Convert ros message to opencv's image
    # -------------------------------------
    if bridge is None:
        bridge = cv_bridge.CvBridge()  # create a cv bridge if none is given

    image = bridge.imgmsg_to_cv2(msg)  # extract image from ros msg
    image = cv2.pyrDown(image)
    image = cv2.pyrDown(image)

    # -------------------------------------
    # Step 2: Initialization
    # -------------------------------------
    now = rospy.Time.now()
    labels = {}
    kernel = np.ones((6, 6), np.uint8)
    propagation_threshold = 600
    height, width = image.shape
    getLinearIndex = functools.partial(getLinearIndexWidth, width=width)
    print('Image size is ' + str(height) + ', ' + str(width))

    if debug:
        cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
        imageShowUInt16OrFloat32OrBool(image, 'Original')
        cv2.namedWindow('DiffUp', cv2.WINDOW_NORMAL)
        cv2.namedWindow('DiffDown', cv2.WINDOW_NORMAL)
        cv2.namedWindow('DiffLeft', cv2.WINDOW_NORMAL)
        cv2.namedWindow('DiffRight', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Seeds', cv2.WINDOW_NORMAL)
        # cv2.namedWindow('Visited', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Propagated', cv2.WINDOW_NORMAL)

    # -------------------------------------
    # Step 3: Preprocessing the image
    # -------------------------------------
    if image.dtype == np.float32:
        cv_image_array = convertDepthImage32FC1to16UC1(image)
        cv2.normalize(cv_image_array, cv_image_array, 0, 65535, cv2.NORM_MINMAX)
    else:
        cv_image_array = np.array(image)
        cv2.normalize(cv_image_array, cv_image_array, 0, 65535, cv2.NORM_MINMAX)
        cv_image_array[np.where((cv_image_array > [20000]))] = [0]
        cv_image_array[np.where((cv_image_array == [0]))] = [2500]
        cv2.normalize(cv_image_array, cv_image_array, 0, 65535, cv2.NORM_MINMAX)

    cv_image_array = cv2.GaussianBlur(cv_image_array, (5, 5), 0)
    img_closed = cv2.morphologyEx(cv_image_array, cv2.MORPH_CLOSE, kernel)

    print('Time taken in preprocessing ' + str((rospy.Time.now() - now).to_sec()))

    # -------------------------------------
    # Step 3: Flood fill (using np arrays)
    # -------------------------------------
    now = rospy.Time.now()

    # x, y = 672, 141
    # x, y = 372, 76
    x, y = 180, 35
    # x, y = 0, 0
    # pix = image[y, x]
    # seed_points = {getLinearIndex(x, y): {'x': x, 'y': y, 'pix': image[y, x]}}

    # TODO This is not integrated with the preprocessing
    # neighbor_up = np.zeros((3, 3), dtype=bool)
    # neighbor_up[0, 1] = True
    #
    # neighbor_down = np.zeros((3, 3), dtype=bool)
    # neighbor_down[2, 1] = True
    #
    # neighbor_right = np.zeros((3, 3), dtype=bool)
    # neighbor_right[1, 2] = True
    #
    # neighbor_left = np.zeros((3, 3), dtype=bool)
    # neighbor_left[1, 0] = True
    # # print(neighbor_up)
    #
    # # array([[7, 1, 7, 8, 3, 6],
    # #        [8, 0, 5, 3, 5, 3],
    # #        [9, 3, 2, 4, 3, 9],
    # #        [2, 9, 6, 9, 4, 6]])
    # # >> > np.diff(A, axis=0)
    # # array([[1, -1, -2, -5, 2, -3],
    # #        [1, 3, -3, 1, -2, 6],
    # #        [-7, 6, 4, 5, 1, -3]])
    # # print(image.dtype)
    # # print(cv_image_array.dtype)
    #
    # propagation_threshold = 0.05
    #
    # diff_up = np.absolute(np.diff(image, axis=0))
    # diff_up = np.vstack((diff_up, np.ones((1, width)).astype(image.dtype) * propagation_threshold))
    # diff_up = (diff_up < propagation_threshold).astype(bool)
    # imageShowUInt16OrFloat32OrBool(diff_up, 'DiffUp')
    #
    # diff_down = np.absolute(np.diff(image, axis=0))
    # diff_down = np.vstack((np.ones((1, width)).astype(image.dtype) * propagation_threshold + 1, diff_down))
    # diff_down = (diff_down < propagation_threshold).astype(bool)
    # imageShowUInt16OrFloat32OrBool(diff_down, 'DiffDown')
    #
    # diff_right = np.absolute(np.diff(image, axis=1))
    # diff_right = np.hstack((diff_right, np.ones((height, 1)).astype(image.dtype) * propagation_threshold))
    # diff_right = (diff_right < propagation_threshold).astype(bool)
    # imageShowUInt16OrFloat32OrBool(diff_right, 'DiffRight')
    #
    # diff_left = np.absolute(np.diff(image, axis=1))
    # diff_left = np.hstack((np.ones((height, 1)).astype(image.dtype) * propagation_threshold, diff_left))
    # diff_left = (diff_left < propagation_threshold).astype(bool)
    # imageShowUInt16OrFloat32OrBool(diff_left, 'DiffLeft')
    #
    # # diff_down = np.diff(image, axis=0)
    # # diff_right = np.diff(image, axis=1)
    # # diff_left = -np.diff(image, axis=1)
    #
    # # cv2.waitKey(0)
    # # print('this is diff_up')
    # # print(diff_up)
    # # from matplotlib import pyplot as plt
    # # plt.imshow(diff_up, cmap='gray')
    # # plt.figure()
    # # plt.imshow(diff_left, cmap='gray')
    # # plt.show()
    #
    # seed_mask = np.zeros(image.shape, dtype=bool)
    # seed_mask[y, x] = True
    #
    # visited_mask = np.zeros(image.shape, dtype=bool)
    # visited_mask[y, x] = True
    # propagated_mask = np.zeros(image.shape, dtype=bool)
    # propagated_mask[y, x] = True
    #
    # area = 0
    # area_prev = 1
    # while not area == area_prev:
    #     # up
    #     dilated_mask_up = ndimage.binary_dilation(seed_mask, structure=neighbor_up)
    #     propagated_up = np.logical_and(dilated_mask_up, diff_up)
    #
    #     # down
    #     dilated_mask_down = ndimage.binary_dilation(seed_mask, structure=neighbor_down)
    #     propagated_down = np.logical_and(dilated_mask_down, diff_down)
    #
    #     # right
    #     dilated_mask_right = ndimage.binary_dilation(seed_mask, structure=neighbor_right)
    #     propagated_right = np.logical_and(dilated_mask_right, diff_right)
    #
    #     # left
    #     dilated_mask_left = ndimage.binary_dilation(seed_mask, structure=neighbor_left)
    #     propagated_left = np.logical_and(dilated_mask_left, diff_left)
    #
    #     propagated_mask = np.logical_or.reduce((propagated_up, propagated_down, propagated_right, propagated_left))
    #
    #     # replace seed mask
    #     seed_mask = np.logical_or(seed_mask, propagated_mask)
    #
    #     area_prev = area
    #     area = np.sum(seed_mask)
    #
    #     # seed_mask = np.logical_or(np.logical_or(np.logical_or(propagated_up, propagated_right), propagated_down), propagated_left)
    #
    #
    #
    # print('Time taken in floodfill ' + str((rospy.Time.now() - now).to_sec()))
    #
    # # print(np.sum(propagated_now))
    # imageShowUInt16OrFloat32OrBool(seed_mask, 'Seeds')
    # # imageShowUInt16OrFloat32OrBool(visited_mask, 'Visited')
    # imageShowUInt16OrFloat32OrBool(propagated_mask, 'Propagated')
    # cv2.waitKey(1)
    # now = rospy.Time.now()

    # -------------------------------------
    # Step 3: Flood fill (using lists)
    # -------------------------------------
    now = rospy.Time.now()
    #
    # x, y = 672, 141
    x, y = 180, 35
    # x, y = 0, 0
    # pix = image[y, x]
    seed_points = {getLinearIndex(x, y): {'x': x, 'y': y, 'pix': image[y, x]}}
    visited_mask = np.zeros(image.shape, dtype=bool)
    visited_mask[y, x] = True
    propagated_mask = np.zeros(image.shape, dtype=bool)
    propagated_mask[y, x] = True

    while seed_points:  # while we have a non empty dictionary of seed points, propagate
        point_key = list(seed_points.keys())[0]  # pick up first point in the dict
        point = seed_points[point_key]

        # use a N4 neighbor_up for simplification
        #                     up      down     left    right
        neighbour_deltas = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dy, dx in neighbour_deltas:
            x, y = point['x'] + dx, point['y'] + dy
            if x < 0 or x >= width or y < 0 or y >= height:  # outside the image
                continue
            visited_mask[y, x] = True
            if not propagated_mask[y, x]:  # not visited yet
                pix = image[y, x]
                if abs(float(pix) - float(point['pix'])) < propagation_threshold:  # propagate into this neighbor
                    seed_points[getLinearIndex(x, y)] = {'x': x, 'y': y, 'pix': pix}
                    propagated_mask[y, x] = True

        del seed_points[point_key]

        # imageShowUInt16OrFloat32OrBool(visited_mask, 'Visited')
        # imageShowUInt16OrFloat32OrBool(propagated_mask, 'Propagated')
        # cv2.waitKey(10)

    print('Time taken in floodfill ' + str((rospy.Time.now() - now).to_sec()))

    now = rospy.Time.now()

    # -------------------------------------
    # Step 4: Canny
    # -------------------------------------

    # propagated_mask = propagated_mask.astype(np.uint8) * 255
    fill_holes = ndimage.morphology.binary_fill_holes(propagated_mask)
    fill_holes = fill_holes.astype(np.uint8) * 255  # these are the idxs

    canny = cv2.Canny(fill_holes, 100, 200)  # these are the idxs_limit_points

    # calculate moments of binary image
    M = cv2.moments(fill_holes)

    print('Time taken in canny ' + str((rospy.Time.now() - now).to_sec()))

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    new_seed_point = [cX, cY]
    # result_image = canny.copy()
    # colored_image=image.copy()
    # result_image[propagated_mask == 255] = 255
    # cv2.cvtColor(image, colored_image)
    # result_image[canny == 255] = [0,255,0]
    # cv2.drawContours(image, [c], -1, (0, 255, 0), 2)

    # new seed point will be the centroid of the detected chessboard in that image ... Assuming that the movement is smooth and the processing fast enough for this to work

    return labels, canny, new_seed_point
