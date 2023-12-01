import math
from copy import copy, deepcopy

import cv2
import numpy as np
import scipy.spatial.distance
from atom_calibration.collect.label_messages import *
from atom_core.dataset_io import getMsgAndCvImageFromDictionaryDepth
from sqlalchemy import true
from atom_core.drawing import drawSquare2D


def normalizeDepthImage(image, max_value=5):
    height, width = image.shape

    # Removing nans before casting to uint8
    # https://github.com/lardemua/atom/issues/643
    image_without_nans = np.nan_to_num(image)

    gui_image = np.zeros((height, width, 3), dtype=np.uint8)
    gui_image[:, :, 0] = image_without_nans / max_value * 255
    gui_image[:, :, 1] = image_without_nans / max_value * 255
    gui_image[:, :, 2] = image_without_nans / max_value * 255
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
                           depth_mode, args, tolerance_radius=20):

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
        tic = rospy.Time.now()

        if depth_mode['mode'] == 'delete':
            print('Deleting depth boundary inside polygon ...')

            height = dataset['sensors'][sensor_key]['camera_info']['height']
            width = dataset['sensors'][sensor_key]['camera_info']['width']
            pattern_mask = getMaskFromPoints(clicked_points[collection_key][sensor_key]['points'], height, width)

            idxs_to_remove = [] 
            for pattern_key in dataset['calibration_config']['calibration_patterns'].keys():
                for idx, linear_idx in enumerate(dataset['collections'][collection_key]['labels'][pattern_key][sensor_key]['idxs_limit_points']):
                    y = int(int(linear_idx) / int(width))
                    x = linear_idx - width * y

                    if pattern_mask[y,x] == 255: # point inside polygon
                        idxs_to_remove.append(idx)

            idxs_to_remove.reverse()
            for idx in idxs_to_remove:
                del dataset['collections'][collection_key]['labels'][sensor_key]['idxs_limit_points'][idx]

            clicked_points[collection_key][sensor_key]['valid_polygon'] = True

            print('Completed deleting depth boundary inside polygon') 


        elif depth_mode['mode'] == 'detect':

            print('Labeling pattern from user defined polygon .. it may take some time ...')
            height = dataset['sensors'][sensor_key]['camera_info']['height']
            width = dataset['sensors'][sensor_key]['camera_info']['width']
            pattern_mask = getMaskFromPoints(clicked_points[collection_key][sensor_key]['points'], height, width)
            msg, image = getMsgAndCvImageFromDictionaryDepth(dataset['collections'][collection_key]['data'][sensor_key])

            # Filter out edges where depth is very far away
            # https://github.com/lardemua/atom/issues/612

            # print('pattern_mask.dtype = ' + str(pattern_mask.dtype))
            # print('type(image) = ' + str(type(image)))
            # print('image.dtype = ' + str(image.dtype))

            # calculate moments of binary image
            M = cv2.moments(pattern_mask)
            
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center = (cX, cY) 
            # put text and highlight the center
    
            pattern_mask_bool = pattern_mask.astype(bool)
            pattern_mask_filtered = deepcopy(pattern_mask)
            
            height, width = image.shape
            for x in range(0, width):
                for y in range(0, height):
                    if pattern_mask[y,x] == 255:

                        # being start and end two points (x1,y1), (x2,y2)
                        discrete_line = list(zip(*line(*center, *(x,y))))

                        ranges = [image[b,a] for a,b in discrete_line]
                        idxs_to_remove = []
                        remove_all = False
                        range_prev = ranges[0]
                        for (x0, y0), (x1, y1) in zip(discrete_line[0:-1], discrete_line[1:]):
                            value0 = image[y0,x0]
                            value1 = image[y1,x1]

                            if np.isnan(value1):
                                continue

                            if np.isnan(value0):
                                value0 = range_prev
                            else:
                                range_prev = value0

                            diff = abs(value1 - value0)
                            if diff > 0.1 or remove_all:
                                # print('removing pixel x=' + str(x1) + ',y=' + str(y1) + ' from mask')
                                pattern_mask_filtered[y1,x1] = 0
                                remove_all = True

                        # cv2.imshow('pattern_mask', pattern_mask)
    #                     gui = normalizeDepthImage(image, max_value=5)
    #                     drawSquare2D(gui, x, y, size=7, color=(50, 190, 0), thickness=2)
    # 
    #                     cv2.circle(gui, (cX, cY), 5, (255, 255, 255), -1)
    #                     # cv2.putText(gui, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255),2)
    # 
    #                     for x,y in discrete_line:
    #                         cv2.line(gui, (x, y), (x, y), (128,0,0), 1)
    # 
                        
                        # cv2.imshow('gui', gui)
                        # cv2.waitKey(10)

            # pattern_mask_removals = (np.logical_xor(pattern_mask.astype(bool), pattern_mask_filtered.astype(bool))).astype(np.uint8)*255
            # cv2.imshow('pattern_mas_removals', pattern_mask_removals)
            # cv2.waitKey(0)
            # print('DONE')
            # new_values = image[pattern_mask_filtered.astype(bool)]
            # with np.printoptions(threshold=np.inf):
            #     print(new_values)
            # # cv2.imwrite('pattern_mask_filtered.png', pattern_mask_filtered)
            # pattern_mask_filtered = cv2.imread('pattern_mask_filtered.png', cv2.IMREAD_GRAYSCALE)
            # cv2.namedWindow('pattern_filtered', cv2.WINDOW_NORMAL)
            # cv2.imshow('pattern_filtered', pattern_mask_filtered)

            # TODO #646 we should use argument filter_border_edges here as well
            labels, gui_image, _ = labelDepthMsg(msg, seed=None, bridge=None,
                                                pyrdown=0, scatter_seed=True,
                                                scatter_seed_radius=8,
                                                debug=False,
                                                subsample_solid_points=7, limit_sample_step=1,
                                                pattern_mask=pattern_mask_filtered,
                                                filter_border_edges=0.025,
                                                remove_nan_border=args['remove_nan_border'])


            # Update the idxs and idxs_limit labels
            # TODO only works for first pattern
            first_pattern_key = list(dataset['calibration_config']['calibration_patterns'].keys())[0]
            dataset['collections'][collection_key]['labels'][first_pattern_key][sensor_key] = labels

            clicked_points[collection_key][sensor_key]['valid_polygon'] = True

            print('Labeling pattern completed in  ' + str((rospy.Time.now() - tic).to_sec()))


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
