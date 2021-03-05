import math
import random
import struct

import scipy
import numpy as np
import atom_core.dataset_io
import ros_numpy


def labelImageMsg():
    pass


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

    # Compute the limit and the middle points for the pattern cluster
    points = np.zeros((4, len(inliers)))
    points[0, :] = inliers[:, 0]
    points[1, :] = inliers[:, 1]
    points[2, :] = inliers[:, 2]
    points[3, :] = 1

    # - Cartesian to polar LiDAR points conversion
    points_sph = []
    # for idx in range(points.shape[0]):
    for idx in range(points.shape[1]):
        # m_pt = points[idx, 0:3]
        m_pt = points[0:3, idx]
        r = math.sqrt(m_pt[0] ** 2 + m_pt[1] ** 2 + m_pt[2] ** 2)
        theta = math.acos(m_pt[2] / r)
        phi = math.atan2(m_pt[1], m_pt[0])

        m_pt_shp = [r, theta, phi]
        points_sph.append(m_pt_shp)

    # - LiDAR beam clustering using the theta component
    points_sph = np.array(points_sph).transpose()
    thetas = points_sph[1, :].round(decimals=4)  # we round so that we can use the np.unique
    unique, indexes, inverse_indexes = np.unique(thetas, return_index=True, return_inverse=True)

    # - Find the extrema points using the maximum and minimum of the phi component for each cluster
    labels['idxs_limit_points'] = []
    labels['idxs_middle_points'] = []
    for i in range(0, len(indexes)):
        m_beam = np.where(inverse_indexes == i)

        phis = points_sph[2, m_beam][0]
        min_idx = np.argmin(phis)
        max_idx = np.argmax(phis)

        for phi in phis:
            if not phi == np.min(phis) and not phi == np.max(phis):
                idx = np.where(points_sph[2, :] == phi)[0][0]
                labels['idxs_middle_points'].append(idx)

        global_min_idx = np.where(points_sph[2, :] == phis[min_idx])[0][0]
        global_max_idx = np.where(points_sph[2, :] == phis[max_idx])[0][0]

        labels['idxs_limit_points'].append(global_min_idx)
        labels['idxs_limit_points'].append(global_max_idx)

    return labels, seed_point, inliers
