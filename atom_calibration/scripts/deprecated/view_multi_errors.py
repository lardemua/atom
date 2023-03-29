#!/usr/bin/env python

import argparse
import json
from collections import OrderedDict
from itertools import permutations

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from OptimizationUtils.tf import TFTree, Transform
from OptimizationUtils.utilities import matrixToRodrigues
from OptimizationUtils import utilities


def load_data(jsonfile):
    try:
        with open(jsonfile, 'r') as f:
            dataset = json.load(f)
    except IOError as e:
        print(str(e))
        exit(1)

    for collection in dataset['collections'].values():
        tree = TFTree()
        for _, tf in collection['transforms'].items():
            param = tf['trans'] + tf['quat']
            tree.add_transform(tf['parent'], tf['child'], Transform(*param))

        collection['tf_tree'] = tree

        dx = dataset['calibration_config']['calibration_pattern']['dimension']["x"]
        dy = dataset['calibration_config']['calibration_pattern']['dimension']["y"]
        size = dataset['calibration_config']['calibration_pattern']['size']
        grid = np.zeros((dx * dy, 4), float)
        grid[:, :2] = size * np.mgrid[0:dx, 0:dy].T.reshape(-1, 2)
        grid[:, 3] = 1
        dataset['grid'] = grid.T

    return dataset

def calc_projection_errors(axis, dataset, from_sensor, to_sensor):

    error = []
    colors = cm.tab20b(np.linspace(0, 1, len(dataset['collections'].items())))
    axis.grid(True)

    sorted_collection = OrderedDict(sorted(dataset['collections'].items(), key=lambda x: int(x[0])))

    for collection_key, collection in sorted_collection.items():
        if not collection['labels'][from_sensor]['detected']:
            continue

        if not collection['labels'][to_sensor]['detected']:
            continue

        tree = collection['tf_tree']
        labels = collection['labels'][from_sensor]

        # 1. Calculate pattern to sensor transformation.
        K = np.ndarray((3, 3), dtype=float, buffer=np.array(dataset['sensors'][from_sensor]['camera_info']['K']))
        D = np.ndarray((5, 1), dtype=float, buffer=np.array(dataset['sensors'][from_sensor]['camera_info']['D']))

        corners = np.zeros((3, len(labels['idxs'])), dtype=float)
        ids = [0] * len(labels['idxs'])
        for idx, point in enumerate(labels['idxs']):
            corners[0, idx] = point['x']
            corners[1, idx] = point['y']
            corners[2, idx] = 1
            ids[idx] = point['id']

        _, rvecs, tvecs = cv2.solvePnP(np.array(dataset['grid'][:3, :].T[ids]), corners[:2,:].T.reshape(-1, 1, 2), K, D)
        sTc = utilities.traslationRodriguesToTransform(tvecs, rvecs)

        # 2. Get the transformation between sensors
        tTf = tree.lookup_transform(dataset['sensors'][from_sensor]['camera_info']['header']['frame_id'],
                                    dataset['sensors'][to_sensor]['camera_info']['header']['frame_id']).matrix

        if from_sensor == to_sensor:
            x1 = tree.lookup_transform(dataset['calibration_config']['calibration_pattern']['link'],
                                       dataset['calibration_config']['world_link']).matrix

            x2 = tree.lookup_transform(dataset['calibration_config']['world_link'],
                                       dataset['sensors'][to_sensor]['camera_info']['header']['frame_id']).matrix

            sTc = np.dot(x2, x1);

        # 3. Transform the corners to the destination sensor
        K2 = np.ndarray((3, 3), dtype=float, buffer=np.array(dataset['sensors'][to_sensor]['camera_info']['K']))
        D2 = np.ndarray((5, 1), dtype=float, buffer=np.array(dataset['sensors'][to_sensor]['camera_info']['D']))

        labels = collection['labels'][to_sensor]
        corners2 = np.zeros((2, len(labels['idxs'])), dtype=float)
        ids2 = [0] * len(labels['idxs'])
        for idx, point in enumerate(labels['idxs']):
            corners2[0, idx] = point['x']
            corners2[1, idx] = point['y']
            ids2[idx] = point['id']

        corners = np.dot(tTf, np.dot(sTc, dataset['grid'].T[ids2].T))
        projected, _, _ = utilities.projectToCamera(K2, D2, 0, 0, corners)

        # 4. Calculate difference
        diff = projected - corners2
        axis.plot(diff[0,:], diff[1,:], 'o', label=collection_key, alpha=0.7, color=colors[int(collection_key)])

        error.extend(np.sum(diff*diff, 0).tolist())

    if from_sensor == to_sensor:
        axis.patch.set_facecolor('blue')
        axis.patch.set_alpha(0.05)

    rmse = (np.sqrt(np.mean(np.array(error))))
    axis.set_title('$e_{\mathrm{rmse}} ' + ' = {:.4}$'.format(rmse))

    lim = 4
    axis.set_ylim(-lim, lim)
    axis.set_xlim(-lim, lim)

    print("From '{}' to '{}'".format(from_sensor, to_sensor))
    print (np.sqrt(np.mean(np.array(error))))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("error", help="Error file", metavar='error_file', type=str)
    parser.add_argument("-t", "--title", help="Plot title", default='Hand-Eye', type=str)
    parser.add_argument("-s", "--save", help="Save plots to file", dest='save', action='store_true')
    args = vars(parser.parse_args())
    name = args['title'].replace(' ', '_').lower()

    dataset = load_data(args['error'])

    snames = dataset['sensors'].keys()
    sidx = {snames[v]: v for v in range(0, len(snames))}
    perm = permutations(snames, 2)


    num_plots = len(snames)
    fig, axes = plt.subplots(num_plots, num_plots, figsize=(15,10))
    # plt.setp(axes.flat, xlabel='$x$ error (pixel)', ylabel='$y$ error (pixel)')

    for names in perm:
        f, t = names
        calc_projection_errors(axes[sidx[f],sidx[t]], dataset, f, t)

    for n in snames:
        calc_projection_errors(axes[sidx[n],sidx[n]], dataset, n, n)

    smap = {'kinect_camera': 'World Camera',
            'astra_camera' : 'Hand Camera',
            'camera0': 'camera0',
            'camera1': 'camera1',
            'camera2': 'camera2' }
    pad = 10 # in points
    for ax, col in zip(axes[0], snames):
        ax.annotate(smap[col], xy=(0.5, 1), xytext=(0, pad*2.5),
        xycoords='axes fraction', textcoords='offset points',
        size='xx-large', ha='center', va='baseline')

    for ax, row in zip(axes[:,0], snames):
        ax.annotate(smap[row], xy=(0, 0.5), xytext=(-ax.yaxis.labelpad - pad, 0),
        xycoords=ax.yaxis.label, textcoords='offset points',
        size='xx-large', ha='right', va='center', rotation=90)

    # handles, labels = axes[num_plots-1, num_plots-1].get_legend_handles_labels()
    # fig.legend(handles, labels, ncol=1, loc='upper right', bbox_to_anchor=(1, 0.96), title="Collections")

    fig.tight_layout()
    fig.subplots_adjust(left=0.1, top=0.94)

    fig.savefig('rmse-matrix.pdf')

    plt.show()



if __name__ == '__main__':
    main()
