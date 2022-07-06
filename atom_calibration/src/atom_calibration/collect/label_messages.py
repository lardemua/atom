
# Standard imports
import copy
import math
import random

import scipy
import numpy as np
import cv2

# ROS imports
import cv_bridge
import rospy
import ros_numpy
from scipy import ndimage
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image

# Atom imports


def labelImageMsg():
    pass
    # TODO the stuff from patterns.py should be here


def find_nearest_white(img, target):
    nonzero = np.argwhere(img == 255)
    distances = np.sqrt((nonzero[:, 0] - target[0]) ** 2 + (nonzero[:, 1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index]


def labelPointCloud2Msg(msg, seed_x, seed_y, seed_z, threshold, ransac_iterations,
                        ransac_threshold):
    n_inliers = 0
    labels = {}

    pc = ros_numpy.numpify(msg)

    # Compute number of points by multiplying all the dimensions of the np array.
    # Must be done because different lidars provide point clouds with different sizes, e.g. velodyne outputs a point cloud of size (npoints,1), whereas ouster lidars output a point cloud of size (npoints_per_layer, nlayers).
    #More info https://github.com/lardemua/atom/issues/498
    number_points = 1
    for value in list(pc.shape):
        number_points = number_points*value

    points = np.zeros((number_points, 3))
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
    if number_points < 10:
        labels = {'detected': False, 'idxs': [], 'idxs_limit_points': [], 'idxs_middle_points': []}
        seed_point = [seed_x, seed_y, seed_z]
        return labels, seed_point, []

    # RANSAC iterations
    for i in range(0, ransac_iterations):

        # print('ransac iteration ' + str(i))
        # print('number of points ' + str(number_points))
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

    for unique_theta in unique_thetas:
        ps_in_cluster = [item for item in ps if item['theta'] == unique_theta]
        # Get list of points with
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

    cv2.namedWindow(window_name, flags=cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image_scaled)


def convertDepthImage32FC1to16UC1(image_in, scale=1000.0):
    """
    The assumption is that the image_in is a float 32 bit np array, which contains the range values in meters.
    The image_out will be a 16 bit unsigned int  [0, 65536] which will store the range in millimeters.
    As a convetion, nans will be set to the maximum number available for uint16
    :param image_in: np array with dtype float32
    :param scale: convertion from meters to milimieter if 1000
    :return: np array with dtype uint16
    """

    if image_in.dtype == np.uint16:
        raise ValueError("Cannot convert float32 to uint16 because image is already uint16.")
        # return image_in

    # mask_nans = np.isnan(image_in)
    image_mm_float = image_in * scale  # convert meters to millimeters
    image_mm_float = np.round(image_mm_float)  # round the millimeters
    image_out = image_mm_float.astype(np.uint16)  # side effect: nans become 0s
    return image_out


def convertDepthImage16UC1to32FC1(image_in, scale=1000.0):
    """
    The assumption is that the image_in is a uint16 bit np array, which contains the range values in millimeters.
    The image_out will be a float32 which will store the range in meters.
    As a convention, nans will be set from all pixels with the maximum number available for uint16
    :param image_in: np array with dtype  uint16
    :param scale: convertion from millimieters to meters if 1000
    :return: np array with dtype float32
    """
    # iinfo = np.iinfo(np.uint16)
    mask_nans = image_in == 0
    image_out = image_in.astype(np.float32) / scale  # convert millimeters to meters
    image_out[mask_nans] = np.nan
    return image_out


def getLinearIndexWidth(x, y, width):
    """ Gets the linear pixel index given the x,y and the image width """
    return x + y * width


def labelDepthMsg(msg, seed=None, propagation_threshold=0.2, bridge=None, pyrdown=0,
                  scatter_seed=False, scatter_seed_radius=8, subsample_solid_points=1, debug=False,
                  limit_sample_step=5, filter_border_edges=0.025, pattern_mask=None):
    """
    Labels rectangular patterns in ros image messages containing depth images.

    :param msg: An image message with a depth image.
    :param seed: dictionary containing coordinates of seed point
    :param propagation_threshold: maximum value of pixel difference under which propagation occurs.
    :param bridge: a cvbridge data structure to avoid having to constantly create one.
    :param pyrdown: The ammount of times the image must be downscaled using pyrdown. 0 for no pyrdown
    :param scatter_seed: To scatter the given seed in a circle of seed points. Useful because the given seed coordinate
                         may fall under a black rectangle of the pattern.
    :param scatter_seed: The radius of the scatter points
    :param subsample_solid_points: Subsample factor of solid pattern points to go into the output labels.
    :param debug: Debug prints and shows images.
    :param limit_sample_step
    :param filter_border_edges: Percentage of border that is to be ignored if the chessboard touches that area.
    :param pattern_mask: Mask with the pattern already defined. If not None, skips region growing steps in this function.
    :return: labels, a dictionary like this {'detected': True, 'idxs': [], 'idxs_limit_points': []}.
             gui_image, an image for visualization purposes which shows the result of the labeling.
             new_seed_point, pixels coordinates of centroid of the pattern area.
    """

    # Verification
    if pattern_mask is not None:
        assert pattern_mask.dtype == np.uint8, "Pattern_mask is not of np.uint8 type"

    # -------------------------------------
    # Step 1: Convert ros message to opencv's image and downsample if needed, and make sure it is float in meters
    # -------------------------------------
    if bridge is None:  # Convert to opencv image
        bridge = cv_bridge.CvBridge()  # create a cv bridge if none is given
    image = bridge.imgmsg_to_cv2(msg)  # extract image from ros msg

    # TODO test for images that are uint16 (only float was tested)
    if not image.dtype == np.float32:  # Make sure it is float
        image = convertDepthImage16UC1to32FC1(image)
        # print("Image converted to float 32\n")
        # print(image.dtpye)
        # raise ValueError('image must be float type, and it is ' + str(image.dtype))

    original_image = copy.deepcopy(image)

    if pattern_mask is None:
        seed_x = seed['x']
        seed_y = seed['y']
    else:  # When a pattern_mask is defined, we assume the seed is on the center of the polygon for visualization purposes
        # TODO define seed as center of image
        seed_x = int(image.shape[0] / 2)
        seed_y = int(image.shape[1] / 2)

    for idx in range(0, pyrdown):  # Downsample
        if pattern_mask is not None:
            pattern_mask = cv2.pyrDown(pattern_mask)
        image = cv2.pyrDown(image)
        seed_x = int(seed_x / 2)
        seed_y = int(seed_y / 2)

    # print(seed_x, seed_y)
    # -------------------------------------
    # Step 2: Initialization
    # -------------------------------------
    height, width = image.shape
    if pyrdown == 0:
        original_height = height
        original_width = width
    else:
        original_height = height * (2 * pyrdown)
        original_width = width * (2 * pyrdown)

    if debug:
        print('Image size is ' + str(height) + ', ' + str(width))

    if pattern_mask is None:
        # Setup the difference images in all four directions
        # erode to avoid problem discussed in https://github.com/lardemua/atom/issues/323
        neighbors = np.ones((3, 3), dtype=bool)

        diff_up = np.absolute(np.diff(image, axis=0))
        diff_up = np.vstack((diff_up, np.ones((1, width)).astype(image.dtype) * propagation_threshold))
        diff_up = (diff_up < propagation_threshold).astype(bool)
        diff_up = ndimage.binary_erosion(diff_up, structure=neighbors)

        diff_down = np.absolute(np.diff(image, axis=0))
        diff_down = np.vstack((np.ones((1, width)).astype(image.dtype) * propagation_threshold + 1, diff_down))
        diff_down = (diff_down < propagation_threshold).astype(bool)
        diff_down = ndimage.binary_erosion(diff_down, structure=neighbors)

        diff_right = np.absolute(np.diff(image, axis=1))
        diff_right = np.hstack((diff_right, np.ones((height, 1)).astype(image.dtype) * propagation_threshold))
        diff_right = (diff_right < propagation_threshold).astype(bool)
        diff_right = ndimage.binary_erosion(diff_right, structure=neighbors)

        diff_left = np.absolute(np.diff(image, axis=1))
        diff_left = np.hstack((np.ones((height, 1)).astype(image.dtype) * propagation_threshold, diff_left))
        diff_left = (diff_left < propagation_threshold).astype(bool)
        diff_left = ndimage.binary_erosion(diff_left, structure=neighbors)

        # generate seed points in a circle around the give seed point coordinates (to tackle when the coordinate in in a
        # black hole in the pattern).
        if scatter_seed:
            n = 10
            thetas = np.linspace(0, 2 * math.pi, n)
            r = scatter_seed_radius
            initial_seeds = np.zeros((2, n), dtype=np.int)
            for col in range(0, n):
                x = seed_x + r * math.cos(thetas[col])
                y = seed_y + r * math.sin(thetas[col])
                initial_seeds[0, col] = x
                initial_seeds[1, col] = y
        else:
            initial_seeds = np.array([[seed_x], [seed_y]], dtype=np.int)

        to_visit_mask = np.ones(image.shape, dtype=bool)
        seeds_mask = np.zeros(image.shape, dtype=bool)
        seeds = copy.copy(initial_seeds)

        if debug:
            cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
            imageShowUInt16OrFloat32OrBool(image, 'Original')
            # cv2.namedWindow('DiffUp', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('DiffDown', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('DiffLeft', cv2.WINDOW_NORMAL)
            # cv2.namedWindow('DiffRight', cv2.WINDOW_NORMAL)
            cv2.namedWindow('to_visit_mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('seeds_mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('ResultImage', cv2.WINDOW_NORMAL)

        # -------------------------------------
        # Step 3: Flood fill (supersonic mode - hopefully ... )
        # -------------------------------------
        if debug:
            now = rospy.Time.now()

        while True:  # flood fill loop
            # up direction
            seeds_up = copy.copy(seeds)  # make a copy
            seeds_up[1, :] -= 1  # upper row
            mask_up = np.logical_and(diff_up[seeds_up[1, :], seeds_up[0, :]],
                                     to_visit_mask[seeds_up[1, :], seeds_up[0, :]])
            propagated_up = seeds_up[:, mask_up]

            # down direction
            seeds_down = copy.copy(seeds)  # make a copy
            seeds_down[1, :] += 1  # lower row
            mask_down = np.logical_and(diff_down[seeds_down[1, :], seeds_down[0, :]],
                                       to_visit_mask[seeds_down[1, :], seeds_down[0, :]])
            propagated_down = seeds_down[:, mask_down]

            # left direction
            seeds_left = copy.copy(seeds)  # make a copy
            seeds_left[0, :] -= 1  # left column
            mask_left = np.logical_and(diff_left[seeds_left[1, :], seeds_left[0, :]],
                                       to_visit_mask[seeds_left[1, :], seeds_left[0, :]])
            propagated_left = seeds_left[:, mask_left]

            # right direction
            seeds_right = copy.copy(seeds)  # make a copy
            seeds_right[0, :] += 1  # right column
            mask_right = np.logical_and(diff_right[seeds_right[1, :], seeds_right[0, :]],
                                        to_visit_mask[seeds_right[1, :], seeds_right[0, :]])
            propagated_right = seeds_right[:, mask_right]

            # set the seed points for the next iteration
            seeds = np.hstack((propagated_up, propagated_down, propagated_left, propagated_right))
            # print(seeds.shape)
            if seeds.shape[1] != 0:
                seeds = np.unique(seeds, axis=1)  # make sure the points are unique

            # Mark the visited points
            to_visit_mask[seeds_up[1, :], seeds_up[0, :]] = False
            to_visit_mask[seeds_down[1, :], seeds_down[0, :]] = False
            to_visit_mask[seeds_left[1, :], seeds_left[0, :]] = False
            to_visit_mask[seeds_right[1, :], seeds_right[0, :]] = False

            # seeds_mask = np.zeros(image.shape, dtype=bool) # Uncomment this to see the seed points on each iteration
            seeds_mask[seeds[1, :], seeds[0, :]] = True

            if debug:
                # imageShowUInt16OrFloat32OrBool(diff_up, 'DiffUp')
                # imageShowUInt16OrFloat32OrBool(diff_down, 'DiffDown')
                # imageShowUInt16OrFloat32OrBool(diff_right, 'DiffRight')
                # imageShowUInt16OrFloat32OrBool(diff_left, 'DiffLeft')
                imageShowUInt16OrFloat32OrBool(to_visit_mask, 'to_visit_mask')
                imageShowUInt16OrFloat32OrBool(seeds_mask, 'seeds_mask')
                cv2.waitKey(5)

            if not seeds.size:  # termination condition: no more seed points
                break

        if debug:
            print(
                'Time taken in floodfill (using debug=' + str(debug) + ') is ' + str((rospy.Time.now() - now).to_sec()))

    # -------------------------------------
    # Step 4: Postprocess the filled image
    # -------------------------------------
    if debug:
        now = rospy.Time.now()

    if pattern_mask is not None:  # Construction of not NaN mask
        mask_not_nan = np.logical_not(np.isnan(image))
        seeds_mask = np.logical_and(pattern_mask, mask_not_nan)

    pattern_solid_mask = ndimage.morphology.binary_fill_holes(seeds_mask)  # close the holes
    pattern_solid_mask = pattern_solid_mask.astype(np.uint8) * 255  # convert to uint8

    # contours using cv2.findContours
    contours, hierarchy = cv2.findContours(pattern_solid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Create a convex hull of the contours
    # https://theailearner.com/2020/11/03/opencv-minimum-area-rectangle/
    pattern_edges_mask = cv2.Sobel(src=pattern_solid_mask, ddepth=cv2.CV_8UC1, dx=1, dy=1, ksize=3)
    point_coordinates = np.argwhere(pattern_edges_mask == 255)
    convex_hull_points = cv2.convexHull(point_coordinates)

    if not convex_hull_points is None:
        contours = [[]]  # stupid data strucutre just to be the same as in contours ...
        convex_hull_points = convex_hull_points.tolist()
        convex_hull_points.append(convex_hull_points[0])
        # print('convex_hull_points = ' + str(convex_hull_points))
        for idx, (corner_1, corner_2) in enumerate(zip(convex_hull_points, convex_hull_points[1:])):
            # print('corner_1 ' + str(corner_1))z
            # print('line ' + str(idx))
            x1, y1 = corner_1[0]
            x2, y2 = corner_2[0]
            vector_x = x2 - x1
            vector_y = y2 - y1

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            # print('distance ' + str(distance))
            number_of_samples_in_line = int(distance / limit_sample_step)
            # print('number_of_samples_in_line ' + str(number_of_samples_in_line))

            sampled_values = np.linspace(0, 1, number_of_samples_in_line).astype(float).tolist()
            # print('sampled_values ' + str(sampled_values))
            for sampled_value in sampled_values:
                x = int(x1 + sampled_value * vector_x)
                y = int(y1 + sampled_value * vector_y)
                contours[0].append([[y, x]])

    # cv2.imshow('Canny', pattern_edges_mask)
    # cv2.imshow('Canny Edges After Contouring', pattern_edges_mask)
    # print(image.dtype)
    # # image2 = np.zeros(image.shape, dtype=np.float)
    # image2 = image.copy()
    # cv2.drawContours(image2, contours, -1, (0, 255, 0), 3)
    # cv2.imshow('Contours', image2)
    # cv2.waitKey(10)

    if debug:
        cv2.namedWindow('pattern_solid_mask', cv2.WINDOW_NORMAL)
        cv2.imshow('pattern_solid_mask', pattern_solid_mask)
        cv2.namedWindow('pattern_edges_mask', cv2.WINDOW_NORMAL)
        cv2.imshow('pattern_edges_mask', pattern_edges_mask)
        print('Time taken in canny ' + str((rospy.Time.now() - now).to_sec()))

    # -------------------------------------
    # Step 5: Compute next iteration's seed point as centroid of filled mask
    # -------------------------------------
    moments = cv2.moments(pattern_solid_mask)  # compute moments of binary image
    m0 = moments["m00"]
    if m0 == 0:  # use center of image as backup value
        cx = int(width / 2)
        cy = int(height / 2)
    else:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])

    if pyrdown > 0:
        cx = cx * (2 * pyrdown)  # compensate the pyr down
        cy = cy * (2 * pyrdown)  # compensate the pyr down

    new_seed_point = {'x': cx, 'y': cy}  # pythonic

    # -------------------------------------
    # Step 6: Creating the output labels dictionary
    # -------------------------------------
    labels = {'detected': True, 'idxs': [], 'idxs_limit_points': []}

    if m0 == 0 or not contours:  # if no detection occurred
        labels['detected'] = False
    else:
        # The coordinates in the labels must be given as linear indices of the image in the original size. Because of
        # this we must take into account if some pyrdown was made to recover the coordinates of the original image.
        # Also, the solid mask coordinates may be subsampled because they contain a lot of points.

        # Solid mask coordinates
        sampled = np.zeros(image.shape, dtype=bool)
        sampled[::subsample_solid_points, ::subsample_solid_points] = True
        idxs_rows, idxs_cols = np.where(np.logical_and(pattern_solid_mask, sampled))

        if pyrdown > 0:
            idxs_rows = idxs_rows * (2 * pyrdown)  # compensate the pyr down
            idxs_cols = idxs_cols * (2 * pyrdown)

        # Filter out NaN values in the original image
        labels['idxs'] = []
        for row, col in zip(list(idxs_rows), list(idxs_cols)):
            if not np.isnan(original_image[row, col]):
                # store linear index
                labels['idxs'].append(col + original_width * row)

        # Edges mask coordinates computed from the contours
        idxs_rows = []
        idxs_cols = []
        external_contour = contours[0]
        for value in external_contour:
            x = value[0][0]
            y = value[0][1]

            if np.isnan(image[y, x]):  # if x,y is nan in depth image, find nearest white pixel
                closest_not_nan_pixel = find_nearest_white(pattern_solid_mask, (y, x))
                x = closest_not_nan_pixel[1]
                y = closest_not_nan_pixel[0]

            if x < (width - 1 - (width - 1) * filter_border_edges) and x > (width - 1) * filter_border_edges:
                if y < (height - 1 - (height - 1) * filter_border_edges) and y > (height - 1) * filter_border_edges:
                    idxs_rows.append(y)
                    idxs_cols.append(x)

        idxs_rows = np.array(idxs_rows)
        idxs_cols = np.array(idxs_cols)

        if pyrdown > 0:
            idxs_rows = idxs_rows * (2 * pyrdown)  # compensate the pyr down
            idxs_cols = idxs_cols * (2 * pyrdown)

        # check if point is in the image's limits
        idxs = idxs_cols + original_width * idxs_rows  # we will store the linear indices
        idxs = idxs.tolist()
        idxs = idxs[::limit_sample_step]  # subsample the limit points
        labels['idxs_limit_points'] = idxs

    # -------------------------------------
    # Step 7: Creating the output image
    # -------------------------------------
    now = rospy.Time.now()
    # Create a nice color image as result
    gui_image = np.zeros((height, width, 3), dtype=np.uint8)
    max_value = 5
    gui_image[:, :, 0] = image / max_value * 255
    gui_image[:, :, 1] = image / max_value * 255
    gui_image[:, :, 2] = image / max_value * 255

    if labels['detected']:
        if pattern_mask is None:
            # "greenify" the filled region
            gui_image[seeds_mask, 0] -= 10
            gui_image[seeds_mask, 1] += 10
            gui_image[seeds_mask, 2] -= 10
            gui_image = gui_image.astype(np.uint8)

        # show subsampled points
        for idx in labels['idxs']:
            y = int(idx / original_width)
            x = int(idx - y * original_width)

            if pyrdown > 0:
                y = int(y / (2 * pyrdown))
                x = int(x / (2 * pyrdown))

            gui_image[y, x, 0] = 0
            gui_image[y, x, 1] = 200
            gui_image[y, x, 2] = 255

        # # Draw the Canny boundary
        # mask_canny = pattern_edges_mask.astype(bool)
        # gui_image[mask_canny, 0] = 25
        # gui_image[mask_canny, 1] = 255
        # gui_image[mask_canny, 2] = 25

        # Draw the subsampled limit points
        for idx in labels['idxs_limit_points']:
            y = int(idx / original_width)
            x = int(idx - y * original_width)

            if pyrdown > 0:
                y = int(y / (2 * pyrdown))
                x = int(x / (2 * pyrdown))

            cv2.line(gui_image, (x, y), (x, y), (255, 0, 200), 3)

    # Initial seed point as red dot
    cv2.line(gui_image, (seed_x, seed_y), (seed_x, seed_y), (0, 0, 255), 3)

    # initial seeds scatter points
    if scatter_seed and pattern_mask is None:
        for col in range(0, initial_seeds.shape[1]):
            x = initial_seeds[0, col]
            y = initial_seeds[1, col]
            cv2.line(gui_image, (x, y), (x, y), (0, 0, 255), 3)

    # next seed point as blue dot
    if pyrdown > 0:
        cx = int(cx / (2 * pyrdown))
        cy = int(cy / (2 * pyrdown))
    cv2.line(gui_image, (cx, cy), (cx, cy), (255, 0, 0), 3)

    # new seed point will be the centroid of the detected chessboard in that image ... Assuming that the movement is
    # smooth and the processing fast enough for this to work

    if debug:
        cv2.imshow('ResultImage', gui_image)
        cv2.waitKey(5)
        print('Time taken preparing gui image ' + str((rospy.Time.now() - now).to_sec()))

    return labels, gui_image, new_seed_point


def calculateFrustrum(w, h, f_x, f_y, Z_near, Z_far, frame_id, ns, color):
    marker = Marker()

    marker.ns = ns
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.header.frame_id = frame_id
    # marker scale
    marker.scale.x = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    P1 = Point()
    P2 = Point()
    P3 = Point()
    P4 = Point()
    P5 = Point()
    P6 = Point()
    P7 = Point()
    P8 = Point()

    # Z_near = 0.3
    # Z_far = 8
    fov_x = 2 * math.atan2(w, (2 * f_x))
    fov_y = 2 * math.atan2(h, (2 * f_y))

    x_n = math.tan(fov_x / 2) * Z_near
    y_n = math.tan(fov_y / 2) * Z_near

    x_f = math.tan(fov_x / 2) * Z_far
    y_f = math.tan(fov_y / 2) * Z_far

    P1.x = -x_n
    P1.y = y_n
    P1.z = Z_near

    P2.x = x_n
    P2.y = y_n
    P2.z = Z_near

    P3.x = x_n
    P3.y = -y_n
    P3.z = Z_near

    P4.x = -x_n
    P4.y = -y_n
    P4.z = Z_near

    P5.x = -x_f
    P5.y = y_f
    P5.z = Z_far

    P6.x = x_f
    P6.y = y_f
    P6.z = Z_far

    P7.x = x_f
    P7.y = -y_f
    P7.z = Z_far

    P8.x = -x_f
    P8.y = -y_f
    P8.z = Z_far

    # marker line points
    marker.points = []

    marker.points.append(P1)
    marker.points.append(P2)

    marker.points.append(P2)
    marker.points.append(P3)

    marker.points.append(P3)
    marker.points.append(P4)

    marker.points.append(P4)
    marker.points.append(P1)

    marker.points.append(P1)
    marker.points.append(P5)
    #
    marker.points.append(P2)
    marker.points.append(P6)

    marker.points.append(P3)
    marker.points.append(P7)

    marker.points.append(P4)
    marker.points.append(P8)

    marker.points.append(P5)
    marker.points.append(P6)

    marker.points.append(P6)
    marker.points.append(P7)

    marker.points.append(P7)
    marker.points.append(P8)

    marker.points.append(P8)
    marker.points.append(P5)

    return marker


def getFrustumMarkerArray(w, h, f_x, f_y, Z_near, Z_far, frame_id, ns, color):
    # big help from https: // github.com/ros-visualization/rviz/issues/925
    marker_array = MarkerArray()

    # ------------------------------------
    # Define view frustum points
    # ------------------------------------
    fov_x = 2 * math.atan2(w, (2 * f_x))
    fov_y = 2 * math.atan2(h, (2 * f_y))

    x_n = math.tan(fov_x / 2) * Z_near
    y_n = math.tan(fov_y / 2) * Z_near

    x_f = math.tan(fov_x / 2) * Z_far
    y_f = math.tan(fov_y / 2) * Z_far

    points = [Point(-x_n, y_n, Z_near),
              Point(x_n, y_n, Z_near),
              Point(x_n, -y_n, Z_near),
              Point(-x_n, -y_n, Z_near),
              Point(-x_f, y_f, Z_far),
              Point(x_f, y_f, Z_far),
              Point(x_f, -y_f, Z_far),
              Point(-x_f, -y_f, Z_far)]

    # ------------------------------------
    # Define wireframe
    # ------------------------------------
    color_rviz = ColorRGBA(r=color[0]/2, g=color[1]/2, b=color[2]/2, a=1.0)
    marker = Marker(ns=ns+'_wireframe', type=Marker.LINE_LIST, action=Marker.ADD, header=Header(frame_id=frame_id),
                    color=color_rviz)

    marker.scale.x = 0.005  # line width
    marker.pose.orientation.w = 1.0

    # marker line points
    marker.points.append(points[0])
    marker.points.append(points[1])

    marker.points.append(points[1])
    marker.points.append(points[2])

    marker.points.append(points[2])
    marker.points.append(points[3])

    marker.points.append(points[3])
    marker.points.append(points[0])

    marker.points.append(points[0])
    marker.points.append(points[4])

    marker.points.append(points[1])
    marker.points.append(points[5])

    marker.points.append(points[2])
    marker.points.append(points[6])

    marker.points.append(points[3])
    marker.points.append(points[7])

    marker.points.append(points[4])
    marker.points.append(points[5])

    marker.points.append(points[5])
    marker.points.append(points[6])

    marker.points.append(points[6])
    marker.points.append(points[7])

    marker.points.append(points[7])
    marker.points.append(points[4])

    marker_array.markers.append(copy.deepcopy(marker))

    # ------------------------------------
    # Define filled
    # ------------------------------------
    color_rviz = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.5)
    marker = Marker(ns=ns+'_filled', type=Marker.TRIANGLE_LIST, action=Marker.ADD, header=Header(frame_id=frame_id),
                    color=color_rviz)

    marker.scale.x = 1  # line width
    marker.scale.y = 1  # line width
    marker.scale.z = 1  # line width
    marker.pose.orientation.w = 1.0

    # marker triangles of the lateral face of the frustum pyramid
    marker.points.append(points[1])
    marker.points.append(points[2])
    marker.points.append(points[6])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[1])
    marker.points.append(points[6])
    marker.points.append(points[5])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[0])
    marker.points.append(points[4])
    marker.points.append(points[3])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[3])
    marker.points.append(points[4])
    marker.points.append(points[7])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[0])
    marker.points.append(points[1])
    marker.points.append(points[5])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[0])
    marker.points.append(points[4])
    marker.points.append(points[5])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[3])
    marker.points.append(points[2])
    marker.points.append(points[6])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker.points.append(points[3])
    marker.points.append(points[6])
    marker.points.append(points[7])
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)
    marker.colors.append(color_rviz)

    marker_array.markers.append(copy.deepcopy(marker))

    return marker_array


def worldToPix(fx, fy, cx, cy, X, Y, Z):
    x_pix = (fx * X + cx * Z) / Z
    y_pix = (fy * Y + cy * Z) / Z
    return x_pix, y_pix


def pixToWorld(fx, fy, cx, cy, x_pix, y_pix, z):
    x_over_z = (x_pix - cx) / fx
    y_over_z = (y_pix - cy) / fy
    x = x_over_z * z
    y = y_over_z * z
    return x, y, z
