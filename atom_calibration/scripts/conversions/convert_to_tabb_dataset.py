#!/usr/bin/env python
import shutil

import atom_core.atom
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
    # ap.add_argument("-b", "--base", help="This problem is an eye-to-base.", dest="base", action="store_true")
    # ap.add_argument("-s", "--sensor", help="This problem uses a single sensor. This argument defines its name.",
    #                 type=str, required=True)
    args = vars(ap.parse_args())

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

        shutil.rmtree(args['dataset_out'])  # Delete old folder
        os.makedirs(args['dataset_out'])
    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    """ Loads a json file containing the detections"""
    with open(args['json_file'], 'r') as f:
        dataset_sensors = json.load(f)

    # print(dataset_sensors)
    # print("\n")

    # create folders for each sensor to put images in
    for sensor_key, sensor_name in enumerate(dataset_sensors['sensors']):
        folder = args['dataset_out'] + '/images/' + sensor_name
        if not os.path.exists(folder):
            print('Creating folder ' + folder)
            os.makedirs(folder)  # Create the new folder

    # Copy images from input to output dataset.
    print('Copying ' + str(len(dataset_sensors['collections'].keys())) + ' images...')
    keys = [int(key) for key in dataset_sensors['collections'].keys()]  # get keys as integers
    keys.sort()  # sort integer keys
    sorted_keys = [str(key) for key in keys]  # convert back to string from sorted integers

    for sensor_key, sensor_name in enumerate(dataset_sensors['sensors']):
        for collection_key in sorted_keys:
            collection = dataset_sensors['collections'][collection_key]
            filename = os.path.dirname(args['json_file']) + '/' + collection['data'][sensor_name]['data_file']
            _, extension = os.path.splitext(os.path.basename(filename))
            filename_out = args['dataset_out'] + '/images/' + sensor_name + '/image' + str(collection_key) + extension

            copyfile(filename, filename_out)

    # Create the calibration_object.txt file
    filename = args['dataset_out'] + '/' + 'calibration_object.txt'
    with open(filename, 'w') as file_handle:
        file_handle.write('chess_mm_height ' + str(
            float(dataset_sensors['calibration_config']['calibration_pattern']['size']) * 1000) + '\n')
        file_handle.write(
            'chess_mm_width ' + str(float(dataset_sensors['calibration_config']['calibration_pattern']['size']) * 1000) + '\n')
        file_handle.write(
            'chess_height ' + str(dataset_sensors['calibration_config']['calibration_pattern']['dimension']['y']) + '\n')
        file_handle.write(
            'chess_width ' + str(dataset_sensors['calibration_config']['calibration_pattern']['dimension']['x']) + '\n')
        file_handle.close()
        print('Created  ' + filename + '.')


    # Create the ExperimentDetails.txt file
    filename = args['dataset_out'] + '/' + 'ExperimentDetails.txt'
    with open(filename, 'w') as file_handle:
        # TODO Eurico, we should have a field for the description of the experiment in our json file. You agree?
        file_handle.write('File created automatically, no details added.')
        print('Created  ' + filename + '.')

    # Create the robot_cali.txt file
    filename = args['dataset_out'] + '/' + 'robot_cali.txt'
    with open(filename, 'w') as file_handle:
        # must travel the dictionary in order
        keys = [int(key) for key in dataset_sensors['collections'].keys()]  # get keys as integers
        keys.sort()  # sort integer keys
        sorted_keys = [str(key) for key in keys]  # convert back to string from sorted integers

        number_of_images = len(sorted_keys)
        file_handle.write(str(number_of_images) + '\n')

        for collection_key in sorted_keys:
            collection = dataset_sensors['collections'][collection_key]

            # TODO Eurico, I think you inverted the transform in the opposite translation, perhaps we need to do it
            #  here too?
            T = atom_core.atom.getTransform('ee_link', dataset_sensors['calibration_config']['world_link'], collection['transforms'])
            # T = utilities.getTransform(dataset_sensors['calibration_config']['world_link'], 'ee_link', collection['transforms'])

            T[0,3] = T[0,3] * 1000.0
            T[1,3] = T[1,3] * 1000.0
            T[2,3] = T[2,3] * 1000.0

            print('Collection ' + collection_key + ' =\n' + str(T))
            H, W = T.shape
            for h in range(0, H):
                for w in range(0, W):
                    file_handle.write(str(T[h, w]) + ' ')

                file_handle.write('\n')

            file_handle.write('\n')

        file_handle.close()
