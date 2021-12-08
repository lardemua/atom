import copy
import math
import random
import struct

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
        image_scaled = image_scaled*255

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

    # -------------------------------------
    # Step 2: Initialization
    # -------------------------------------
    now = rospy.Time.now()
    labels = {}
    kernel = np.ones((6, 6), np.uint8)
    propagation_threshold = 200
    height, width = image.shape
    getLinearIndex = functools.partial(getLinearIndexWidth, width=width)
    print('Image size is ' + str(height) + ', ' + str(width))

    if debug:
        cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
        imageShowUInt16OrFloat32OrBool(image, 'Original')
        cv2.namedWindow('Visited', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Propagated', cv2.WINDOW_AUTOSIZE)

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
    # Step 3: Flood fill
    # -------------------------------------
    now = rospy.Time.now()

    x, y = 672, 141
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

        # use a N4 neighborhood for simplification
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

        imageShowUInt16OrFloat32OrBool(visited_mask, 'Visited')
        imageShowUInt16OrFloat32OrBool(propagated_mask, 'Propagated')
        cv2.waitKey(10)

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
