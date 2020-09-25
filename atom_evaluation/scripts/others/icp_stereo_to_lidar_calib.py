#!/usr/bin/env python

"""
Stereo calibration from opencv
"""

# -------------------------------------------------------------------------------
# --- IMPORTS
# -------------------------------------------------------------------------------

import atom_core.atom
import numpy as np
import json
import argparse
from pcl import PointCloud


def load(path, format=None):
    """Load pointcloud from path.
    Currently supports PCD and PLY files.
    Format should be "pcd", "ply", or None to infer from the pathname.
    """
    format = _infer_format(path, format)
    p = PointCloud()
    try:
        loader = getattr(p, "_from_%s_file" % format)
    except AttributeError:
        raise ValueError("unknown file format %s" % format)
    if loader(_encode(path)):
        raise IOError("error while loading pointcloud from %r (format=%r)"
                      % (path, format))
    return p

def _encode(path):
    # Encode path for use in C++.
    if isinstance(path, bytes):
        return path
    else:
        return path.encode(sys.getfilesystemencoding())


def _infer_format(path, format):
    if format is not None:
        return format.lower()

    for candidate in ["pcd", "ply", "obj"]:
        if path.endswith("." + candidate):
            return candidate

    raise ValueError("Could not determine file format from pathname %s" % path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-json", "--json_file", help="Json file containing initial guess estimate.", type=str,
                    required=True)
    ap.add_argument("-ss", "--source_sensor", help="Source transformation sensor.", type=str, required=True)
    ap.add_argument("-ts", "--target_sensor", help="Target transformation sensor.", type=str, required=True)
    ap.add_argument("-fpcl", "--first_pcl", help="First point cloud from the two to be aligned", type=str,
                    required=True)
    ap.add_argument("-spcl", "--second_pcl", help="Second point cloud from the two to be aligned", type=str,
                    required=True)

    # Save args
    args = vars(ap.parse_args())
    fpcl_path = args['first_pcl']
    spcl_path = args['second_pcl']
    source_sensor = args['source_sensor']
    target_sensor = args['target_sensor']

    # Load input json file
    json_file = args['json_file']
    f = open(json_file, 'r')
    dataset = json.load(f)

    # Load point clouds from files
    fpcl = load(fpcl_path)
    spcl = load(spcl_path)

    # Get initial guess
    selected_collection_key = dataset['collections'].keys()[0]
    from_frame = dataset['calibration_config']['sensors'][target_sensor]['link']
    to_frame = dataset['calibration_config']['sensors'][source_sensor]['link']
    vel2cam = atom_core.atom.getTransform(from_frame, to_frame,
                                          dataset['collections'][selected_collection_key]['transforms'])

    # Transform source point cloud using initial guess
    fpcl_h = np.zeros((fpcl.size, 4), np.float32)
    fpcl_h[:, 0:3] = fpcl
    fpcl_h[:, 3] = 1
    fpcl_h = np.dot(fpcl_h, vel2cam)
    fpcl_h = fpcl_h[:, 0:3]
    fpcl = PointCloud(fpcl_h.astype(np.float32))

    # Apply icp alignment
    print ('Starting ICP alignemnt ...')
    icp = fpcl.make_IterativeClosestPoint()
    converged, transf, estimate, fitness = icp.icp(fpcl, spcl)

    print('Has converged:' + str(converged) + ', score: ' + str(fitness))
    print('Result:\n' + str(transf))