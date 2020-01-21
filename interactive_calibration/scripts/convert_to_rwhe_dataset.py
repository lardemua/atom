#!/usr/bin/env python
import sys
import os.path
import argparse

import json
from colorama import Fore, Style

# ROS imports
from shutil import copyfile

import OptimizationUtils.utilities as utilities

if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-out", "--dataset_out", type=str, required=True, help="Full path to the output dataset folder")
    ap.add_argument("-s", "--sensor", help="This problem uses a single sensor. This argument defines its name.",
                    type=str, required=True)
    args = vars(ap.parse_args())

    # print('Copying images ...')
    # Testing output folder
    if not os.path.exists(args['dataset_out']):
        os.mkdir(args['dataset_out'])  # Create the new folder
    else:
        while True:
            msg = Fore.YELLOW + "To continue, the directory '{}' will be deleted.\n"
            msg = msg + "Do you wish to continue? [y/N] " + Style.RESET_ALL

            answer = raw_input(msg.format(args['dataset_out']))
            if len(answer) > 0 and answer[0].lower() in ('y', 'n'):
                if answer[0].lower() == 'n':
                    sys.exit(1)
                else:
                    break
            else:
                sys.exit(1)  # defaults to N

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    """ Loads a json file containing the detections"""
    with open(args['json_file'], 'r') as f:
        dataset_sensors = json.load(f)

    print(dataset_sensors)
    print("\n")

    # Copy images from input to output dataset.
    print('Copying ' + str(len(dataset_sensors['collections'].keys())) + ' images...')
    keys = [int(key) for key in dataset_sensors['collections'].keys()]  # get keys as integers
    keys.sort()  # sort integer keys
    sorted_keys = [str(key) for key in keys]  # convert back to string from sorted integers

    for collection_key in sorted_keys:
        collection = dataset_sensors['collections'][collection_key]
        filename = os.path.dirname(args['json_file']) + '/' + collection['data'][args['sensor']]['data_file']
        filename_out = args['dataset_out'] + '/' + collection['data'][args['sensor']]['data_file']
        copyfile(filename, filename_out)

    # Create the squaresize.txt file
    filename = args['dataset_out'] + '/' + 'squaresize.txt'
    with open(filename, 'w') as file_handle:
        file_handle.write(str(dataset_sensors['calibration_config']['calibration_pattern']['size']) + 'm')
        file_handle.close()
        print('Created  ' + filename + '.')

    # Create the RobotPosesVec.txt file
    filename = args['dataset_out'] + '/' + 'RobotPosesVec.txt'
    with open(filename, 'w') as file_handle:
        # must travel the dictionary in order
        keys = [int(key) for key in dataset_sensors['collections'].keys()]  # get keys as integers
        keys.sort()  # sort integer keys
        sorted_keys = [str(key) for key in keys]  # convert back to string from sorted integers

        for collection_key in sorted_keys:
            collection = dataset_sensors['collections'][collection_key]
            # T = utilities.getTransform('ee_link', 'base_link', collection['transforms']).reshape((1, 16))
            T = utilities.getTransform('base_link', 'ee_link', collection['transforms']).reshape((1, 16))
            print('Collection ' + collection_key + ' =\n' + str(T))
            h, w = T.shape
            for i in range(0, w):
                file_handle.write(str(T[0, i]) + ' ')

            file_handle.write('\n')


        file_handle.close()
