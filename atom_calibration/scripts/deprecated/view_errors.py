#!/usr/bin/env python

import argparse
import json

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from collections import OrderedDict

from OptimizationUtils.tf import Transform


def load_data(jsonfile):
    try:
        with open(jsonfile, 'r') as f:
            dataset = json.load(f)
    except IOError as e:
        print(str(e))
        exit(1)

    return OrderedDict(sorted(dataset['collections'].items(), key=lambda x: int(x[0])))


def calculate_errors(data):

    cerr = []
    terr = []
    for key, collection in data.items():
        for sensor_name, value in collection.items():
            A = Transform(*value['A'])  # tTb
            X = Transform(*value['X'])  # bTw
            Z = Transform(*value['Z'])  # tTc
            B = Transform(*value['B'])  # cTw

            Ra = A.rotation_matrix
            ta = np.array(list(A.position))
            Rx = X.rotation_matrix
            tx = np.array(list(X.position))
            Rz = Z.rotation_matrix
            tz = np.array(list(Z.position))
            Rb = B.rotation_matrix
            tb = np.array(list(B.position))

            # Rotation error
            r = np.dot(np.linalg.inv(np.dot(Rz, Rb)), np.dot(Ra, Rx))

            # diff = matrixToRodrigues(r)
            diff = np.array(Transform.from_matrix(r).euler)
            err = diff*diff
            cerr.append(err.tolist())

            diff = (np.dot(Ra[0:3, 0:3], tx) + ta) - (np.dot(Rz[0:3, 0:3], tb) + tz)
            terr.append(diff)

            # diff = np.dot(A.matrix, X.matrix) - np.dot(Z.matrix, B.matrix)
            # err = np.linalg.norm(diff, ord='fro')
            # cerr.append(err.tolist())

    cerr = np.array(cerr)
    # print(np.mean(np.sqrt(np.sum(cerr,0))))

    terr = np.array(terr)
    # print("Angular error {}".format(np.mean(np.sqrt(np.sum(cerr*cerr, 1))) * 180.0 / 3.14159) )
    print("Angular error {}".format(np.mean(np.sqrt(np.sum(cerr, 1))) * 180.0 / np.pi))
    print("Translation error {}".format(np.mean(np.sqrt(np.sum(terr*terr, 1)))*1000.0))
    # print("Full error {}".format(np.mean(cerr)))


def get_projection_errors(data, error_key):

    all = []
    per_collection = OrderedDict()
    for key, collection in data.items():
        local = []

        per_collection[key] = {'sensors': {}}
        for sensor_name, value in collection.items():
            if value['errors'] is []:
                continue

            errors = np.array(value[error_key])
            error = np.apply_along_axis(np.linalg.norm, 0, errors)**2

            all.extend(error.tolist())
            local.extend(error.tolist())

            xerr = errors[0, :]
            yerr = errors[1, :]

            per_collection[key]['sensors'][sensor_name] = dict(xerr=xerr, yerr=yerr, error=error)

        if local == []:
            continue

        per_collection[key]['error'] = local

    return all, per_collection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("error", help="Error file", metavar='error_file', type=str)
    parser.add_argument("-t", "--title", help="Plot title", default='Hand-Eye', type=str)
    parser.add_argument("-s", "--save", help="Save plots to file", dest='save', action='store_true')
    args = vars(parser.parse_args())
    name = args['title'].replace(' ', '_').lower()

    data = load_data(args['error'])

    # snames = data.values()[0].keys()

    all, per_collection = get_projection_errors(data, 'errors')

    print("Errors for {} collections!".format(len(data)))

    calculate_errors(data)

    sensor_names = data.values()[0].keys()

    all, per_collection = get_projection_errors(data, 'errors')

    # print(np.mean(np.sqrt(all)))
    rmse = np.sqrt(np.mean(all))

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 5))

    y = np.array([[np.sqrt(np.mean(xx['error'])) for xx in x['sensors'].values()] for x in per_collection.values()])

    ret = axes[0].plot(y, '-o')
    axes[0].legend(ret, sensor_names, loc="upper right")
    axes[0].grid(True)
    axes[0].set_ylabel('RMSE')
    axes[0].set_title('RMSE per collection per sensor')

    y = [np.sqrt(np.mean(x['error'])) for x in per_collection.values()]
    axes[1].plot(y, '-o', label='combined')
    axes[1].legend(loc='upper right')
    axes[1].grid(True)
    axes[1].set_ylabel('RMSE')
    axes[1].set_xlabel('# Collection')

    axes[1].set_title('RMSE per collection')

    plt.xticks(range(len(y)), per_collection.keys(), rotation=45)

    fig.tight_layout()

    st = fig.suptitle('{} ($RMSE = {}$)'.format(args['title'], rmse), fontsize=16)
    st.set_y(0.98)
    fig.subplots_adjust(top=0.85)

    if args['save']:
        fig.savefig(name + '_rmse.png')

    plt.show()

    # =========================================

    colors = cm.tab20b(np.linspace(0, 1, len(per_collection)))
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    y = np.array([[xx['yerr'] for xx in x['sensors'].values()] for x in per_collection.values()])
    x = np.array([[xx['xerr'] for xx in x['sensors'].values()] for x in per_collection.values()])

    if x.shape[1] > 1:
        xmean = np.mean(np.concatenate(x.ravel()))
        xstd = np.std(np.concatenate(x.ravel()))

        ymean = np.mean(np.concatenate(y.ravel()))
        ystd = np.std(np.concatenate(y.ravel()))
    else:
        xmean = np.mean(x.ravel())
        xstd = np.std(x.ravel())

        ymean = np.mean(y.ravel())
        ystd = np.std(y.ravel())

    dev = 4
    axes[1].set_title("Final")
    axes[1].grid(True)
    axes[1].set_xlim(xmean - dev*xstd, xmean + dev*xstd)
    axes[1].set_ylim(ymean - dev*ystd, ymean + dev*ystd)
    # axes[1].set_xscale('log')
    # axes[1].set_yscale('log')

    keys = per_collection.keys()
    for i in range(x.shape[0]):
        axes[1].plot(np.concatenate(x[i]), np.concatenate(y[i]), 'o', label=keys[i], alpha=0.7, color=colors[i])

    axes[1].set_xlabel('$x$ error (pixel)')
    axes[1].set_ylabel('$y$ error (pixel)')

    # axes.set_aspect('equal', 'box')

    axes[1].legend(ncol=2, fontsize='xx-small', title='Collections')

    all, per_collection = get_projection_errors(data, 'init_errors')

    y = np.array([[xx['yerr'] for xx in x['sensors'].values()] for x in per_collection.values()])
    x = np.array([[xx['xerr'] for xx in x['sensors'].values()] for x in per_collection.values()])

    if x.shape[1] > 1:
        xmean = np.mean(np.concatenate(x.ravel()))
        xstd = np.std(np.concatenate(x.ravel()))

        ymean = np.mean(np.concatenate(y.ravel()))
        ystd = np.std(np.concatenate(y.ravel()))
    else:
        xmean = np.mean(x.ravel())
        xstd = np.std(x.ravel())

        ymean = np.mean(y.ravel())
        ystd = np.std(y.ravel())

    axes[0].set_title("Initial")
    axes[0].grid(True)

    for i in range(x.shape[0]):
        axes[0].plot(np.concatenate(x[i]), np.concatenate(y[i]), 'o', label=keys[i], alpha=0.7, color=colors[i])

    axes[0].set_xlabel('$x$ error (pixel)')
    axes[0].set_ylabel('$y$ error (pixel)')

    axes[0].set_xlim(xmean - dev*xstd, xmean + dev*xstd)
    axes[0].set_ylim(ymean - 2*dev*ystd, ymean + 2*dev*ystd)

    # axes.set_aspect('equal', 'box')

    axes[0].legend(ncol=2, fontsize='xx-small', title='Collections')

    fig.tight_layout()
    st = fig.suptitle('{} - Reprojection errors'.format(args['title']), fontsize=16)
    st.set_y(0.98)
    fig.subplots_adjust(top=0.85)

    if args['save']:
        fig.savefig(name + '_proj.png')

    plt.show()


if __name__ == '__main__':
    main()
