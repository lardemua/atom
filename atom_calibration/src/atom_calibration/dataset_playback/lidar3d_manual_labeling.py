import sensor_msgs.point_cloud2 as pc2


def selectedPointsCallback(selected_point_cloud, selection, dataset):
    """
    Callback function to add selected points to the dataset as idxs.
    """
    collection_key = selection['collection_key']

    # Extract xyz coordinates from the selected points
    points_selected = pc2.read_points(selected_point_cloud)
    gen_selected_points = list(points_selected)

    if len(gen_selected_points) == 0:  # To prevent unnecessary operations.
        return

    # sensor from the point cloud
    sensor = list(dataset['collections'][collection_key]['labels'].keys())[
        int(gen_selected_points[0][4])]

    selected_idxs = []
    for point in gen_selected_points:
        selected_idxs.append(int(point[3]))

    idx_center = dataset['collections'][collection_key]['labels'][sensor]['idxs']

    [idx_center.append(x) for x in selected_idxs if x not in idx_center]
    dataset['collections'][collection_key]['labels'][sensor]['idxs'] = idx_center

    if dataset['collections'][collection_key]['labels'][sensor]['idxs'] != [] and \
            dataset['collections'][collection_key]['labels'][sensor]['detected'] == False:
        dataset['collections'][collection_key]['labels'][sensor]['detected'] = True


def selectedPointsBorderCallback(selected_point_cloud, selection, dataset):
    """
    Callback function to add selected points to the dataset as idxs_limit_points.
    """
    collection_key = selection['collection_key']

    # Extract xyz coordinates from the selected points
    points_selected = pc2.read_points(selected_point_cloud)
    gen_selected_points = list(points_selected)

    if len(gen_selected_points) == 0:  # To prevent unnecessary operations.
        return

    # sensor from the point cloud
    sensor = list(dataset['collections'][collection_key]['labels'].keys())[
        int(gen_selected_points[0][4])]

    selected_idxs = []
    for point in gen_selected_points:
        selected_idxs.append(int(point[3]))

    idx_center = dataset['collections'][collection_key]['labels'][sensor]['idxs']
    idx_border = dataset['collections'][collection_key]['labels'][sensor]['idxs_limit_points']

    [idx_center.append(x) for x in selected_idxs if x not in idx_center]
    [idx_border.append(x) for x in selected_idxs if x not in idx_border]

    dataset['collections'][collection_key]['labels'][sensor]['idxs'] = idx_center
    dataset['collections'][collection_key]['labels'][sensor]['idxs_limit_points'] = idx_border


def selectedPointsRemoveCallback(selected_point_cloud, selection, dataset):
    """
    Callback function to remove selected points from the dataset.
    """
    collection_key = selection['collection_key']

    # Extract xyz coordinates from the selected points
    points_selected = pc2.read_points(selected_point_cloud)
    gen_selected_points = list(points_selected)

    if len(gen_selected_points) == 0:  # To prevent unnecessary operations.
        return

    # sensor from the point cloud
    sensor = list(dataset['collections'][collection_key]['labels'].keys())[
        int(gen_selected_points[0][4])]

    selected_idxs = []
    for point in gen_selected_points:
        selected_idxs.append(int(point[3]))

    # remove these points from the idx and idxs_limit_points
    idx_center = dataset['collections'][collection_key]['labels'][sensor]['idxs']
    idx_border = dataset['collections'][collection_key]['labels'][sensor]['idxs_limit_points']

    idx_center_new = [x for x in idx_center if x not in selected_idxs]
    idx_border_new = [x for x in idx_border if x not in selected_idxs]

    dataset['collections'][collection_key]['labels'][sensor]['idxs'] = idx_center_new
    dataset['collections'][collection_key]['labels'][sensor]['idxs_limit_points'] = idx_border_new


def selectedPointsClearAllCallback(selected_point_cloud, selection, dataset):
    """
    Callback function to remove all points of the selected sensor from the dataset.
    """
    collection_key = selection['collection_key']
    # Extract xyz coordinates from the selected points
    points_selected = pc2.read_points(selected_point_cloud)
    gen_selected_points = list(points_selected)

    if len(gen_selected_points) == 0:  # To prevent unnecessary operations.
        return

    # sensor from the point cloud
    sensor = list(dataset['collections'][collection_key]['labels'].keys())[
        int(gen_selected_points[0][4])]

    dataset['collections'][collection_key]['labels'][sensor]['idxs'] = []
    dataset['collections'][collection_key]['labels'][sensor]['idxs_limit_points'] = []
