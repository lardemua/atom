import math
from copy import deepcopy, copy

import cv2
import numpy as np
import scipy.spatial.distance

from atom_calibration.collect.label_messages import *
from atom_core.dataset_io import getMsgAndCvImageFromDictionaryDepth


def normalizeDepthImage(image, max_value=5):
    height, width = image.shape
    gui_image = np.zeros((height, width, 3), dtype=np.uint8)
    gui_image[:, :, 0] = image / max_value * 255
    gui_image[:, :, 1] = image / max_value * 255
    gui_image[:, :, 2] = image / max_value * 255
    return gui_image


def drawLabelsOnImage(labels, image, color_idxs=(0, 200, 255), color_idxs_limits=(255, 0, 200)):
    _, width, _ = image.shape

    for idx in labels['idxs']:
        # convert from linear idx to x_pix and y_pix indices.
        y = int(idx / width)
        x = int(idx - y * width)
        cv2.line(image, (x, y), (x, y), color_idxs, 3)

    for idx in labels['idxs_limit_points']:
        # convert from linear idx to x_pix and y_pix indices.
        y = int(idx / width)
        x = int(idx - y * width)
        cv2.line(image, (x, y), (x, y), color_idxs_limits, 3)

    return image


def clickedPointsCallback(point_msg, clicked_points, dataset, sensor_key, selection,
                          tolerance_radius=5):

    collection_key = selection['collection_key']

    if clicked_points[collection_key][sensor_key]['valid_polygon']:
        clickedPointsReset(clicked_points, collection_key, sensor_key)

    # Add point to list of clicked points
    point = {'x': int(point_msg.point.x), 'y': int(point_msg.point.y)}
    clicked_points[collection_key][sensor_key]['points'].append(point)

    # Retrieving clicked points for the current sensor
    clicked_sensor_points = clicked_points[collection_key][sensor_key]['points']

    if len(clicked_sensor_points) < 3:  # if less than 3 points polygon has no area
        clicked_points[collection_key][sensor_key]['valid_polygon'] = False
        return

    # Compute the distance between the first and last placed points
    start_point = [clicked_sensor_points[0]['x'], clicked_sensor_points[0]['y']]
    end_point = [clicked_sensor_points[-1]['x'], clicked_sensor_points[-1]['y']]
    start_to_end_distance = scipy.spatial.distance.euclidean(start_point, end_point)

    # polygon closed, compute new labels
    if start_to_end_distance < tolerance_radius:
        height = dataset['sensors'][sensor_key]['camera_info']['height']
        width = dataset['sensors'][sensor_key]['camera_info']['width']
        pattern_mask = getMaskFromPoints(clicked_points[collection_key][sensor_key]['points'], height, width)

        msg, _ = getMsgAndCvImageFromDictionaryDepth(dataset['collections'][collection_key]['data'][sensor_key])
        labels, gui_image, _ = labelDepthMsg(msg, seed=None, bridge=None,
                                             pyrdown=0, scatter_seed=True,
                                             scatter_seed_radius=8,
                                             debug=False,
                                             subsample_solid_points=1, limit_sample_step=1,
                                             pattern_mask=pattern_mask)

        # Update the idxs and idxs_limit labels
        dataset['collections'][collection_key]['labels'][sensor_key] = labels

        clicked_points[collection_key][sensor_key]['valid_polygon'] = True


def clickedPointsReset(clicked_points, collection_key, sensor_key):
    clicked_points[collection_key][sensor_key] = {'points': [], 'valid_polygon': False}
    return clicked_points


def getMaskFromPoints(points, image_height, image_width):
    pattern_mask_rgb = np.zeros((image_height, image_width, 3), dtype=np.uint8)

    points_list = []
    for point in points[:-1]:
        point_tuple = (point['x'], point['y'])
        points_list.append(point_tuple)
    points_array = (np.array([points_list]))

    # Fill poly needs an np.array of a list of tuples
    cv2.fillPoly(pattern_mask_rgb, pts=points_array,
                 color=(255, 255, 255))

    pattern_mask, _, _ = cv2.split(pattern_mask_rgb)  # single channel mask
    return pattern_mask
