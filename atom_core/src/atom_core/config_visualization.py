#!/usr/bin/env python
"""
A set of utilities to be used in the optimization algorithms
"""

# -------------------------------------------------------------------------------
# --- IMPORTS (standard, then third party, then my own modules)
# -------------------------------------------------------------------------------

# stdlib
import os
import argparse
from os.path import exists
import numpy as np

import matplotlib


# 3rd-party
import numpy
import rospkg
import rosbag
from pytictoc import TicToc

# local packages
from atom_core.utilities import atomPrintOK, atomError, atomStartupPrint
from atom_core.system import execute
from colorama import Style, Fore
from graphviz import Digraph
import networkx as nx
import graphviz
from urdf_parser_py.urdf import URDF
from matplotlib import cm

import atom_core.config_io
import atom_core.drawing


# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def is_world_link(link, config):
    if config['world_link'] == link:
        return True
    else:
        return False


def has_pattern_link(link, config):
    for pattern_key, pattern in config['calibration_patterns'].items():
        if link == pattern['link']:
            return pattern_key

    return None


def has_sensor_data(link, config):
    for sensor_key, sensor in config['sensors'].items():
        if link == sensor['link']:
            return sensor_key

    return None


def is_transformation_calibrated(parent, child, config):
    for sensor_key, sensor in config['sensors'].items():
        if parent == sensor['parent_link'] and child == sensor['child_link']:
            return True

    if config['additional_tfs'] is not None:
        for additional_tf_key, additional_tf in config['additional_tfs'].items():
            if parent == additional_tf['parent_link'] and child == additional_tf['child_link']:
                return True

    return False


def get_joint_name(parent, child, description):
    for joint in description.joints:
        if parent == joint.parent and child == joint.child:
            return joint.name

    return None


def joint_params_calibrated(joint_name, config):

    if config['joints'] is not None:
        for joint_key, joint in config['joints'].items():
            # Search for a joint with the same name in the urdf
            if joint_key == joint_name:
                return joint['params_to_calibrate']

    return None


def createNxGraph(args, description, config, bag):
    nx_graph = nx.DiGraph()
    if not args['use_tfs']:  # create a graph using the information in the urdf
        print('Creating transformation tree using the urdf robot description ...')

        for link in description.links:  # A graph node for each link in the urdf
            nx_graph.add_node(link.name,
                              is_world=is_world_link(link.name, config),
                              pattern=has_pattern_link(link.name, config),
                              sensor_data=has_sensor_data(link.name, config))

        # Graph node for each calibration pattern
        for pattern_key, pattern in config['calibration_patterns'].items():
            nx_graph.add_node(pattern['link'],
                              is_world=is_world_link(pattern['link'], config),
                              pattern=has_pattern_link(pattern['link'], config),
                              sensor_data=has_sensor_data(pattern['link'], config))

        # Graph edges from joints in description
        for joint in description.joints:
            parent = joint.parent
            child = joint.child
            nx_graph.add_edge(parent, child, weight=1, type=joint.type,
                              parent=parent, child=child,
                              is_transformation_calibrated=is_transformation_calibrated(parent, child, config),
                              joint_params_calibrated=joint_params_calibrated(joint.name, config),
                              joint_name=joint.name)

        # Graph edges from calibration patterns
        for pattern_key, pattern in config['calibration_patterns'].items():
            if pattern['fixed']:
                edge_type = 'fixed'
            else:
                edge_type = 'multiple'

            parent = pattern['parent_link']
            child = pattern['link']
            nx_graph.add_edge(parent, child, weight=1, type=edge_type,
                              parent=parent, child=child,
                              is_transformation_calibrated=True,
                              joint_params_calibrated=joint_params_calibrated(
                                  get_joint_name(parent, child, description), config),
                              joint_name=None)

    else:
        print('Creating transformation tree using the tfs in the bagfile ...')

        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                if not nx_graph.has_edge(transform.header.frame_id.replace('/', ''), transform.child_frame_id.replace('/', '')):
                    # print(transform.header.frame_id, transform.child_frame_id)
                    nx_graph.add_edge(transform.header.frame_id, transform.child_frame_id, weight=1, type='dynamic')

        for topic, msg, t in bag.read_messages(topics=['/tf_static']):
            for transform in msg.transforms:
                if not nx_graph.has_edge(transform.header.frame_id.replace('/', ''), transform.child_frame_id.replace('/', '')):
                    # print(transform.header.frame_id.replace('/',''), transform.child_frame_id.replace('/',''))
                    nx_graph.add_edge(transform.header.frame_id.replace('/', ''),
                                      transform.child_frame_id.replace('/', ''), weight=1, type='fixed')

    return nx_graph


def createDotGraph(nx_graph, config):

    dot_graph = Digraph('G', filename='summary', directory='/tmp/', graph_attr={'title': 'graph'})  # create a graph

    colormap = cm.Pastel1(np.linspace(0, 1, 8))
    # https://matplotlib.org/stable/users/explain/colors/colormaps.html
    color_grey = matplotlib.colors.rgb2hex([0.6, 0.6, 0.6])
    color_world_link = matplotlib.colors.rgb2hex(colormap[1, :])
    color_sensor_data = matplotlib.colors.rgb2hex(colormap[2, :])
    color_pattern = matplotlib.colors.rgb2hex(colormap[5, :])
    color_transform_optimized = matplotlib.colors.rgb2hex([0.8, 0, 0])
    color_joint_optimized = matplotlib.colors.rgb2hex([0.94, 0.43, 0])

    for node_key, node in nx_graph.nodes().items():  # Define the graphviz nodes

        # Define the label and color per node
        label = node_key

        if node['is_world']:
            rgb = color_world_link
            label += '\n(world link)'
        elif not node['sensor_data'] is None:
            rgb = color_sensor_data
            label += '\n(data from sensor ' + node['sensor_data'] + ')'
        elif not node['pattern'] is None:
            rgb = color_pattern
            label += '\n(' + config['calibration_patterns'][node['pattern']]['pattern_type'] + ' calibration pattern)'
        else:
            rgb = matplotlib.colors.rgb2hex([0, 0, 0])

        # Create the node
        dot_graph.node(node_key, label=label, _attributes={'penwidth': '2', 'color': rgb}, )

    for edge_key, edge in nx_graph.edges().items():  # Define the graphviz nodes

        # print('\nedge ' + str(edge_key) + ':' + str(edge))
        rgb = matplotlib.colors.rgb2hex([0, 0, 0])
        rgb_font = matplotlib.colors.rgb2hex([0, 0, 0])

        if edge['type'] == 'fixed' and edge['is_transformation_calibrated']:
            label = ' static\n calibrate transform'
            rgb_font = rgb = color_transform_optimized

        elif edge['type'] == 'multiple' and edge['is_transformation_calibrated']:
            label = ' dynamic\n calibrate multiple transforms'
            rgb_font = rgb = color_transform_optimized

        elif edge['type'] == 'revolute' and edge['joint_params_calibrated']:
            label = ' ' + edge['joint_name'] + '\n (revolute)\n calibrate ' + str(edge['joint_params_calibrated'])
            rgb_font = matplotlib.colors.rgb2hex([0.6, 0, 0.6])
            rgb_font = rgb = color_joint_optimized

        elif edge['type'] == 'prismatic' and edge['joint_params_calibrated']:
            label = ' ' + edge['joint_name'] + '\n (prismatic)\n calibrate ' + str(edge['joint_params_calibrated'])
            rgb_font = rgb = color_joint_optimized

        elif edge['type'] == 'continuous' and edge['joint_params_calibrated']:
            label = ' ' + edge['joint_name'] + '\n (continuous)\n calibrate ' + str(edge['joint_params_calibrated'])
            rgb_font = rgb = color_joint_optimized

        elif edge['type'] == 'revolute':
            label = edge['joint_name'] + '\n (revolute)'
            rgb_font = color_grey

        elif edge['type'] == 'prismatic':
            label = edge['joint_name'] + '\n (prismatic)'
            rgb_font = color_grey

        elif edge['type'] == 'continuous':
            label = edge['joint_name'] + '\n (continuous)'
            rgb_font = color_grey

        elif edge['type'] == 'fixed':
            label = ' static'
            rgb_font = color_grey
        else:
            label = 'Unknown case'
            rgb_font = matplotlib.colors.rgb2hex([1, 0, 0])

        dot_graph.edge(edge['parent'], edge['child'], color=rgb, style='solid',
                       _attributes={'penwidth': '1', 'fontcolor': rgb_font},
                       label=label)

    return dot_graph
