#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import json
import math
import os
from functools import partial
import cv2
import numpy as np
from interactive_markers.menu_handler import MenuHandler
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import KeyPressManager.KeyPressManager
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import interactive_calibration.utilities

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------


server = None
menu_handler = MenuHandler()

# ------------------------
#      FUNCTIONS         #
# ------------------------


if __name__ == "__main__":
    # Parse command line arguments
    ap = argparse.ArgumentParser()

    ap.add_argument('-d', '--data_json', help='Output folder to where the collected data will be stored.', type=str,
                    required=True)
    args = vars(ap.parse_args())
    args['dataset_folder'] = os.path.dirname(args['data_json'])
    print('Inferred dataset folder to be: ' + args['dataset_folder'])

    # Reading json file
    fid = open(args['data_json'])
    dataset = json.load(fid)
    # print(json.dumps(dataset, indent=4, sort_keys=True))
    sensors = [i for i in dataset['sensors']]
    print('Dataset contains ' + str(len(sensors)) + ' sensors.')


    def pickCallback(event, sensor_name):
        print('onpick1 event for sensor ' + sensor_name)

        if isinstance(event.artist, Line2D):
            thisline = event.artist
            xdata = thisline.get_xdata()
            ydata = thisline.get_ydata()
            ind = event.ind
            print 'X=' + str(np.take(xdata, ind)[0])  # Print X point
            print 'Y=' + str(np.take(ydata, ind)[0])  # Print Y point


    # Cycle all data stamps and for each show all sensors
    for data_idx, data in enumerate(dataset['data']):
        print('Data stamp ' + str(data_idx))
        labels = {}

        fig_handles = []
        for sensor_idx, sensor in enumerate(sensors):
            print('Sensor ' + str(sensor['_name']) + ' has msg type ' + sensor['msg_type'])
            sensor_data = data[sensor['_name']]

            window_name = str(sensor['_name']) + ' - data stamp ' + str(data_idx)
            fig = plt.figure(sensor_idx)
            fig_handles.append(fig)
            fig.canvas.set_window_title(window_name)
            sensor_labels = {}

            if sensor['msg_type'] == 'Image':

                image = cv2.imread(args['dataset_folder'] + '/' + sensor_data['data_file'])  # read image from disk
                # TODO cvtcolor only if image has 3 channels
                image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # cv2.imshow(window_name, cv_image)

                # Find chessboard corners
                found, corners = cv2.findChessboardCorners(image_gray, (5, 4))
                cv2.drawChessboardCorners(image, (5, 4), corners, found)  # Draw and display the corners

                if found is True:
                    plt.suptitle('Chessboard automatically detected!', color='green')
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(image_gray, corners, (5, 4), (-1, -1), criteria)
                    corners2_d = []
                    for i in corners2:
                        print i
                        corners2_d.append({'x':i[0][0], 'y':i[0][1]})
                    sensor_labels[sensor['_name']] = corners2_d
                    # print('corners=' + str(corners))
                    # print('corners2=' + str(corners2))
                    print(sensor_labels)
                else:
                    plt.suptitle('Chessboard Not detected! Please select chessboard corners manually', color='red')


                plt.imshow(image, picker=True)

                fig.canvas.mpl_connect('pick_event', partial(pickCallback, sensor_name=str(sensor['_name'])))
                plt.show(block=False)

            elif sensor['msg_type'] == 'LaserScan':

                # Draw axes' arrows and labels
                interactive_calibration.utilities.draw_2d_axes(plt)

                # Draw rings around sensor origin
                interactive_calibration.utilities.draw_concentric_circles(
                    plt, radii=[1, 5, 10], color='tab:gray')

                # Plot laser scan in 2D window
                x, y = interactive_calibration.utilities.laser_scan_data_to_xy(sensor_data)
                plt.plot(x, y, '.--', color='black', picker=True)

                fig.canvas.mpl_connect('pick_event', partial(pickCallback, sensor_name=str(sensor['_name'])))

                plt.show(block=False)

        wm = KeyPressManager.KeyPressManager.WindowManager(fig_handles)
        if wm.waitForKey(None, verbose=False):
            exit(0)
