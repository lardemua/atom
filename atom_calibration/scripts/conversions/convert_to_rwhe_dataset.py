#!/usr/bin/env python3
import atom_core.atom
import sys
import os.path
import argparse
import shutil
from scipy.io import savemat

import json
from colorama import Fore, Style

# ROS imports
from atom_core.dataset_io import loadResultsJSON, filterCollectionsFromDataset


if __name__ == "__main__":

    # Parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing input dataset.", type=str, required=True)
    ap.add_argument("-out", "--dataset_out", type=str, required=True, help="Full path to the output dataset folder")
    ap.add_argument("-e", "--eye", help="This problem uses a single sensor. This argument defines its name.",
                    type=str, required=True)
    ap.add_argument("-sf", "--source_frame", help="This problem requires a source frame. This argument defines its name.",
                    type=str, required=True)
    ap.add_argument("-tf", "--target_frame", help="This problem requires a target frame. This argument defines its name.",
                    type=str, required=True)
    ap.add_argument("-csf", "--collection_selection_function", default=None, type=lambda s: eval(s, globals()),
                    help="A string to be evaluated into a lambda function that receives a collection name as input and "
                    "returns True or False to indicate if the collection should be loaded (and used in the "
                    "optimization). The Syntax is lambda name: f(x), where f(x) is the function in python "
                    "language. Example: lambda name: int(name) > 5 , to load only collections 6, 7, and onward.")
    ap.add_argument("-uic", "--use_incomplete_collections", action="store_true", default=False,
                    help="Remove any collection which does not have a detection for all sensors.", )
    ap.add_argument("-rpd", "--remove_partial_detections", help="Remove detected labels which are only partial."
                            "Used or the Charuco.", action="store_true", default=False)
    ap.add_argument("-sd", "--save_detections", action="store_true", default=False,
                    help="Save ATOM datasets RGB pattern detection in txt form.", )
    args = vars(ap.parse_args())

    # print('Copying images ...')
    # Testing output folder
    if not os.path.exists(args['dataset_out']):
        os.mkdir(args['dataset_out'])  # Create the new folder
    else:
        while True:
            msg = Fore.YELLOW + "To continue, the directory '{}' will be deleted.\n"
            msg = msg + "Do you wish to continue? [y/N] " + Style.RESET_ALL

            answer = input(msg.format(args['dataset_out']))
            if len(answer) > 0 and answer[0].lower() in ('y', 'n'):
                if answer[0].lower() == 'n':
                    sys.exit(1)
                else:
                    shutil.rmtree(args['dataset_out'])
                    os.mkdir(args['dataset_out'])  # Create the new folder
                    break
            else:
                sys.exit(1)  # defaults to N

    # ---------------------------------------
    # --- INITIALIZATION Read data from file
    # ---------------------------------------
    dataset, json_file = loadResultsJSON(args['json_file'], args["collection_selection_function"])

    # ---------------------------------------
    # --- Filter some collections and / or sensors from the dataset
    # ---------------------------------------
    dataset = filterCollectionsFromDataset(dataset, args)  # filter collections


    # Copy images from input to output dataset.
    print('Copying ' + str(len(dataset['collections'].keys())) + ' images...')

    for collection_key in dataset['collections'].keys():
        collection = dataset['collections'][collection_key]
        filename_in = os.path.dirname(args['json_file']) + '/' + collection['data'][args['eye']]['data_file']
        filename_out = args['dataset_out'] + '/' + collection['data'][args['eye']]['data_file']
        shutil.copyfile(filename_in, filename_out)

    # Create the squaresize.txt file
    squaresize_txt = args['dataset_out'] + '/' + 'squaresize.txt'
    # TODO only works for first pattern
    first_pattern_key = list(dataset['calibration_config']['calibration_patterns'].keys())[0]
    with open(squaresize_txt, 'w') as file_handle:
        file_handle.write(str(dataset['calibration_config']['calibration_patterns'][first_pattern_key]['size']) + 'm')
        print('Created  ' + squaresize_txt + '.')

    # Create the RobotPosesVec.txt and RobotPoses.mat files
    squaresize_txt = args['dataset_out'] + '/' + 'RobotPosesVec.txt'
    tf_mat = []
    with open(squaresize_txt, 'w') as file_handle:
        for collection_key in dataset['collections'].keys():
            collection = dataset['collections'][collection_key]
            T = atom_core.atom.getTransform(args['source_frame'], args['target_frame'], collection['transforms']).reshape((1, 16))
            tf_mat.append(T[0])
            h, w = T.shape
            for i in range(0, w):
                file_handle.write(str(T[0, i]) + ' ')

            file_handle.write('\n')


        file_handle.close()

    savemat(args['dataset_out'] + '/' + 'RobotPoses.mat', {'handposes':tf_mat}) 

    # Create detections txt files
    if not args['save_detections']:
        exit()

    detections_dir = args['dataset_out'] + '/detections'
    # Create the directory if it doesn't exist
    os.makedirs(detections_dir, exist_ok=True)
    # Iterate through collections in the dataset
    for collection_key, collection in dataset['collections'].items():
        # Define the file path for the current collection
        detections_file = os.path.join(detections_dir, f'{collection_key}.txt')

        # Open the file for writing
        with open(detections_file, 'w') as file_handle:
            # Iterate through detections in the current collection
            for detection in collection['labels'][first_pattern_key][args['eye']]['idxs']:
                # Write the detection coordinates to the file
                file_handle.write(f'{detection["x"]} {detection["y"]}\n')    
