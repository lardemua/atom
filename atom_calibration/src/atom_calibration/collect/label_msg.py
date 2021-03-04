
import numpy as np
import atom_core.dataset_io


def labelImageMsg():
    pass


def labelPointCloud2Msg(msg):
    labels = {}  # the labels dictionary for this msg. This will be the returned info
    # should go into collection['labels'][sensor_key]
    import ros_numpy
    cloud_msg = atom_core.dataset_io.getPointCloudMessageFromDictionary(msg)

    # ------------------------------------------------------------------------------------------------
    # -------- Extract the labelled LiDAR points on the pattern
    # ------------------------------------------------------------------------------------------------
    idxs = labels['idxs']
    pc = ros_numpy.numpify(cloud_msg)[idxs]
    points = np.zeros((pc.shape[0], 4))
    points[:, 0] = pc['x']
    points[:, 1] = pc['y']
    points[:, 2] = pc['z']
    points[:, 3] = 1

    labels['labelled_points'] = []
    for idx in range(0, points.shape[0]):
        labels['labelled_points'].append(
            {'x': points[idx, 0], 'y': points[idx, 1], 'z': points[idx, 2],
             'w': points[idx, 3]})

    # - Cartesian to polar LiDAR points conversion
    import math
    points_sph = []
    for idx in range(points.shape[0]):
        m_pt = points[idx, 0:3]
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
    extrema_points = []
    middle_points = []
    for i in range(0, len(indexes)):
        m_beam = np.where(inverse_indexes == i)

        phis = points_sph[2, m_beam][0]
        min_idx = np.argmin(phis)
        max_idx = np.argmax(phis)

        for phi in phis:
            if not phi == np.min(phis) and not phi == np.max(phis):
                idx = np.where(points_sph[2, :] == phi)[0][0]
                middle_points.append(points[idx, :])

        global_min_idx = np.where(points_sph[2, :] == phis[min_idx])[0][0]
        global_max_idx = np.where(points_sph[2, :] == phis[max_idx])[0][0]

        extrema_points.append(points[global_min_idx, :])
        extrema_points.append(points[global_max_idx, :])

    # Save extrema points in a dictionary
    collection['labels'][sensor_key]['limit_points'] = []
    extrema_points = np.array(extrema_points)
    for idx in range(0, len(extrema_points)):
        collection['labels'][sensor_key]['limit_points'].append(
            {'x': extrema_points[idx, 0], 'y': extrema_points[idx, 1], 'z': extrema_points[idx, 2],
             'w': extrema_points[idx, 3]})
    collection['labels'][sensor_key]['middle_points'] = []
    middle_points = np.array(middle_points)
    for idx in range(0, len(middle_points)):
        collection['labels'][sensor_key]['middle_points'].append(
            {'x': middle_points[idx, 0], 'y': middle_points[idx, 1], 'z': middle_points[idx, 2],
             'w': middle_points[idx, 3]})
