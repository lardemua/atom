import math
import random
import struct

import scipy
import numpy as np
import atom_core.dataset_io
import ros_numpy


def labelImageMsg():
    pass
    # TODO the stuff from patterns.py should be here


def labelPointCloud2Msg(msg, seed_x, seed_y, seed_z, threshold, ransac_iterations,
                        ransac_threshold):
    n_inliers = 0
    labels = {}

    pc = ros_numpy.numpify(msg)
    points = np.zeros((pc.shape[0], 3))
    points[:, 0] = pc['x']
    points[:, 1] = pc['y']
    points[:, 2] = pc['z']

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
