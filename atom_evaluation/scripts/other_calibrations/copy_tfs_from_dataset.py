#!/usr/bin/env python3

"""
Copies a given fixed (doesn't vary according to the collection) TF from a dataset to another.
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import argparse
import json
import os
import sys
from atom_core.utilities import atomError, createLambdaExpressionsForArgs
import copy

from colorama import Style, Fore
from atom_core.dataset_io import copyTFToDataset, saveAtomDataset


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    # ---------------------------------------
    # Command line arguments
    # ---------------------------------------
    ap.add_argument("-sd", "--source_dataset", help="Json file containing the source dataset, from which the pattern TFs are copied.", type=str,
                    required=True)
    ap.add_argument("-td", "--target_dataset", help="Json file containing the target dataset.", type=str, required=True)
    ap.add_argument("-pll", "--parent_link_list", required=True, type=str, help="The names of the parent frames in the TF to copy", nargs='+')
    ap.add_argument("-cll", "--child_link_list", required=True, type=str, help="The names of the child frames in the TF to copy", nargs='+')

    # Save args
    # args = vars(ap.parse_known_args()[0])
    arglist = [x for x in sys.argv[1:] if not x.startswith("__")]
    args_original = vars(ap.parse_args(args=arglist))  # these args have the selection functions as strings
    args = createLambdaExpressionsForArgs(args_original)  # selection functions are now lambdas

    source_dataset_name = args['source_dataset']
    target_dataset_name = args['target_dataset']

    # ---------------------------------------
    # Dataset loading and preprocessing
    # ---------------------------------------
    
    f = open(source_dataset_name, 'r')
    source_dataset = json.load(f)
    f.close()

    f = open(target_dataset_name, 'r')
    target_dataset = json.load(f)
    f.close()

    new_dataset = copy.deepcopy(target_dataset)

    # ---------------------------------------
    # --- Define selected collection key.
    # ---------------------------------------
    # We only need to get one collection because optimized transformations are static, which means they are the same for all collections. Let's select the first key in the dictionary and always get that transformation.
    selected_collection_key = list(source_dataset["collections"].keys())[0]
    print("Selected collection key is " + str(selected_collection_key))

    # ---------------------------------------
    # Verifications
    # ---------------------------------------

    # Check if parent link list and child link list have the same number of elements
    if len(args["parent_link_list"]) != len(args["child_link_list"]):
        atomError('The parent link and child link lists have a different number of elements. Check your arguments.')

    # Organize the -pll and -cll args into a list of tuples [(parent1,child1), ...]
    parent_child_list = [(args["parent_link_list"][i], args["child_link_list"][i]) for i in range(len(args["parent_link_list"]))]

    # Check if TF exists and if it is fixed; we do this by comparing the TF from one of the collections to each other
    # I think it only makes sense to do this for the sensor dataset

    is_fixed = True

    for parent_child_tuple in parent_child_list:
        tf_name = parent_child_tuple[0] + '-' + parent_child_tuple[1]
        selected_collection_quat = source_dataset["collections"][selected_collection_key]["transforms"][tf_name]["quat"]
        selected_collection_trans = source_dataset["collections"][selected_collection_key]["transforms"][tf_name]["trans"]
        
        print("Selected collection quat (" + tf_name + "):")
        print(selected_collection_quat)
        
        for collection_key, collection in source_dataset["collections"].items():
            if (selected_collection_quat == collection["transforms"][tf_name]["quat"] and selected_collection_trans == collection["transforms"][tf_name]["trans"]):
                continue
            else:
                atomError("Indicated transform " + tf_name + "is not fixed!")
        

    # ---------------------------------------
    # Copy the tfs
    # ---------------------------------------

    for parent_child_tuple in parent_child_list:
        tf_name = parent_child_tuple[0] + '-' + parent_child_tuple[1]
        selected_collection_quat = source_dataset["collections"][selected_collection_key]["transforms"][tf_name]["quat"]
        selected_collection_trans = source_dataset["collections"][selected_collection_key]["transforms"][tf_name]["trans"]
        for collection_key, collection in new_dataset["collections"].items():
            collection["transforms"][tf_name]["quat"] = selected_collection_quat
            collection["transforms"][tf_name]["trans"] = selected_collection_trans

    # Save results to a json file
    base_file_name = target_dataset_name.split('/')[-1].split('.')[0]
    filename_results_json = os.path.dirname(target_dataset_name) + '/' + base_file_name + '_tfs_copied.json'
    saveAtomDataset(filename_results_json, new_dataset, save_data_files=False)
