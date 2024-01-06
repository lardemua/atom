#!/usr/bin/env python3

import argparse
import pymeshlab


def main():

    ap = argparse.ArgumentParser()
    ap.add_argument("-f", "--filename", help='full path to 3d file', type=str, required=True)
    ap.add_argument("-m", "--mass", help='full path to 3d file', type=float, required=True)
    args = vars(ap.parse_args())

    precision = 4
    scale_factor = 100

    ms = pymeshlab.MeshSet()

    ms.load_new_mesh(args['filename'])

    print('Calculating the center of mass')
    geom = ms.get_geometric_measures()
    com = geom['barycenter']

    print('Scaling the mesh')
    ms.compute_matrix_from_scaling_or_normalization(axisx=scale_factor, axisy=scale_factor, axisz=scale_factor)

    print('Generating the convex hull of the mesh')
    ms.generate_convex_hull()  # TODO only if object is not watertight

    print('Calculating inertia tensor')
    geom = ms.get_geometric_measures()
    volume = geom['mesh_volume']
    tensor = geom['inertia_tensor'] / pow(scale_factor, 2) * args['mass'] / volume
    mass = args['mass']

    inertial_xml = f'<inertial>\n  <origin xyz="{com[0]:.{precision}f} {com[1]:.{precision}f} {com[2]:.{precision}f}"/>\n  <mass value="{mass:.{precision}f}"/>\n  <inertia ixx="{tensor[0, 0]:.{precision}f}" ixy="{tensor[1, 0]:.{precision}f}" ixz="{tensor[2, 0]:.{precision}f}" iyy="{tensor[1, 1]:.{precision}f}" iyz="{tensor[1, 2]:.{precision}f}" izz="{tensor[2, 2]:.{precision}f}"/>\n</inertial>'
    print(inertial_xml)


if __name__ == '__main__':
    main()
