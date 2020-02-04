#!/usr/bin/env python

import argparse
import json

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from collections import OrderedDict

from OptimizationUtils.tf import Transform
from OptimizationUtils.utilities import matrixToRodrigues

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
    cerr = []
    for key, collection in data.items():
        for sensor_name, value in collection.items():
            A = Transform(*value['A'])
            X = Transform(*value['X'])
            Z = Transform(*value['Z'])
            B = Transform(*value['B'])

            Ra = A.rotation_matrix
            ta = np.array(list(A.position))
            Rx = X.rotation_matrix
            tx = np.array(list(X.position))
            Rz = Z.rotation_matrix
            tz = np.array(list(Z.position))
            Rb = B.rotation_matrix
            tb = np.array(list(B.position))

            r = np.dot(np.dot(Rz, Rb).T, np.dot(Ra, Rx))

            bb = np.eye(4)
            bb[:3,:3] = r[:3,:3]
            aa = Transform.from_matrix(bb)
            diff = list(aa.euler)
            err = np.linalg.norm(diff)
            cerr.append(err.tolist())

            diff = (((np.dot(Ra[0:3,0:3], tx) + ta)  - (np.dot(Rz[0:3,0:3], tb) + tz))) * 1000.0
            err = np.linalg.norm(diff)
            terr.append(err.tolist())

            diff = np.dot(A.matrix, X.matrix) - np.dot(Z.matrix, B.matrix)
            err = np.linalg.norm(diff, ord='fro')
            cerr.append(err.tolist())


    print(np.mean(cerr) * 180.0 / 3.14159 )
    print(np.mean(terr))
    print(np.mean(cerr))
    # print np.apply_along_axis(np.linalg.norm, 0, terr)


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

# def get_error_evolution(data):

#     all = None
#     for key, collection in data.items():
#         for sensor_name, value in collection.items():
#             if all is None:
#                 all = [[]] * len(value['eofe'])

#             for i, l in enumerate(value['eofe']):
#                 all[i].extend(l)
#                 print l
#                 exit(0)

#     return all


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("error", help="Error file", metavar='error_file', type=str)
    args = vars(parser.parse_args())

    data = load_data(args['error'])

    print("Errors for {} collections!".format(len(data)))

    calculate_errors(data)

    sensor_names = data.values()[0].keys()

    all, per_collection = get_projection_errors(data, 'errors')

    # print(np.mean(np.sqrt(all)))
    rmse = np.sqrt(np.mean(all))

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10,5))

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

    st = fig.suptitle('Eye in Hand ($RMSE = {}$)'.format(rmse), fontsize=16)
    st.set_y(0.98)
    fig.subplots_adjust(top=0.85)

    # fig.savefig("plot.png")
    plt.show()

    #==============================================================
    # all = get_error_evolution(data)
    # fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10,5))

    # # y = np.array([[np.sqrt(np.mean(xx['error'])) for xx in x['sensors'].values()] for x in per_collection.values()])

    # # y = [[xx for xx in x.values()] for x in per_collection.values()]
    # # print y

    # # axes[0].plot(y, '-')
    # for i, error in enumerate(all):

    #     rmse =  np.sqrt(np.mean(np.array(error)**2))

    #         # aa = (len(xx)-1)/12
    #         # select = np.arange(aa-1) * 13 + 2
    #     axes[0].plot(rmse, '-o')
    #     axes[0].grid(True)

    #     # ret = axes[0].plot(y, '-o')

    # axes[0].legend(title="Camera")
    # axes[0].grid(True)
    # axes[0].set_ylabel('RMSE')
    # axes[0].set_title('RMSE per collection per sensor')

    # y = [np.sqrt(np.mean(x['error'])) for x in per_collection.values()]
    # axes[1].plot(y, '-o', label='combined')
    # axes[1].legend(loc='upper right')
    # axes[1].grid(True)
    # axes[1].set_ylabel('RMSE')
    # axes[1].set_xlabel('# Collection')

    # axes[1].set_title('RMSE per collection')

    # plt.xticks(range(len(y)), per_collection.keys(), rotation=45)

    # fig.tight_layout()

    # st = fig.suptitle('Eye in Hand ($RMSE = {}$)'.format(rmse), fontsize=16)
    # st.set_y(0.98)
    # fig.subplots_adjust(top=0.85)

    # plt.show()
    # =============================================================

    colors = cm.tab20b(np.linspace(0, 1, len(per_collection)))
    fig, axes = plt.subplots(1,2, figsize=(10,5))

    y = np.array([[xx['yerr'] for xx in x['sensors'].values()] for x in per_collection.values()])
    x = np.array([[xx['xerr'] for xx in x['sensors'].values()] for x in per_collection.values()])

    axes[1].set_title("Final")
    axes[1].grid(True)

    for i in range(x.shape[0]):
        axes[1].plot(np.concatenate(x[i]), np.concatenate(y[i]), 'o', label=str(i), alpha=0.7, color=colors[i])

    axes[1].set_xlabel('$x$ error (pixel)')
    axes[1].set_ylabel('$y$ error (pixel)')

    # axes.set_aspect('equal', 'box')

    axes[1].legend(ncol=2, fontsize='xx-small', title='Collections')

    all, per_collection = get_projection_errors(data, 'init_errors')

    y = np.array([[xx['yerr'] for xx in x['sensors'].values()] for x in per_collection.values()])
    x = np.array([[xx['xerr'] for xx in x['sensors'].values()] for x in per_collection.values()])

    axes[0].set_title("Initial")
    axes[0].grid(True)

    for i in range(x.shape[0]):
        axes[0].plot(np.concatenate(x[i]), np.concatenate(y[i]), 'o', label=str(i), alpha=0.7, color=colors[i])

    axes[0].set_xlabel('$x$ error (pixel)')
    axes[0].set_ylabel('$y$ error (pixel)')

    # axes.set_aspect('equal', 'box')

    axes[0].legend(ncol=2, fontsize='xx-small', title='Collections')


    fig.tight_layout()
    st = fig.suptitle('Eye to base - Reprojection errors', fontsize=16)
    st.set_y(0.98)
    fig.subplots_adjust(top=0.85)

    fig.savefig("etb.png")

    plt.show()


if __name__ == '__main__':
    main()
