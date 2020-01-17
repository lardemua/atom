#!/usr/bin/env python

# ------------------------
#    IMPORT MODULES      #
# ------------------------
import argparse
import numpy as np
import matplotlib
import rospy
from matplotlib import cm
from tf import TransformListener
from urdf_parser_py.urdf import URDF
import rospkg
from colorama import Fore, Style
from graphviz import Digraph

# ------------------------
#      BASE CLASSES      #
# ------------------------

# ------------------------
#      GLOBAL VARS       #
# ------------------------

# ------------------------
#      FUNCTIONS         #
# ------------------------

if __name__ == "__main__":

    g = Digraph('G', filename='calibration_full')

    # g.node(joint.parent, _attributes={'shape': 'ellipse'})
    # g.node(joint.child, _attributes={'shape': 'ellipse'})
    # g.edge(joint.parent, joint.child, color='black', style='dashed')


    # cmap = cm.Set3(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.Pastel2(np.linspace(0, 1, 2))
    # cmap = cm.viridis(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.brg(np.linspace(0, 1, len(xml_robot.sensors)))


    i = 0;
    black = matplotlib.colors.rgb2hex([0, 0, 1])
    gray = matplotlib.colors.rgb2hex([0.4, 0.4, 0.4])
    g.edge('Reference Sensor', 'Sensor 1', '', color=black, style='solid', _attributes={'penwidth': '1'})
    g.edge('Reference Sensor', 'Sensor 2', '', color=black, style='solid', _attributes={'penwidth': '1'})
    g.edge('Reference Sensor', 'Sensor 3', '', color=black, style='solid', _attributes={'penwidth': '1'})
    g.edge('Sensor 3', 'Sensor 4', '', color=black, style='solid', _attributes={'penwidth': '1'})

    g.edge('Sensor 1', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    g.edge('Sensor 1', 'Sensor 2', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    g.edge('Sensor 1', 'Sensor 3', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    g.edge('Sensor 2', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    g.edge('Reference Sensor', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})


    # g.edge('Reference Sensor', 'Sensor 1', '', color=black, style='solid', _attributes={'penwidth': '1'})
    # g.edge('Reference Sensor', 'Sensor 2', '', color=black, style='solid', _attributes={'penwidth': '1'})
    # g.edge('Reference Sensor', 'Sensor 3', '', color=black, style='solid', _attributes={'penwidth': '1'})
    # g.edge('Reference Sensor', 'Sensor 4', '', color=black, style='solid', _attributes={'penwidth': '1'})
    #
    # g.edge('Sensor 1', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    # g.edge('Sensor 1', 'Sensor 2', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    # g.edge('Sensor 1', 'Sensor 3', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    # g.edge('Sensor 2', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})
    # g.edge('Sensor 3', 'Sensor 4', '', color=gray, style='dashed', _attributes={'penwidth': '.4', 'arrowhead': 'none'})

    # g.edge('Sensor1', 'Sensor2', '',
    #        color=rgb, style='solid', _attributes={'penwidth': '2'})
    # g.edge('Sensor2', 'Sensor3', '',
    #        color=rgb, style='solid', _attributes={'penwidth': '2'})
    # g.edge('Sensor2', 'Sensor4', '',
    #        color=rgb, style='solid', _attributes={'penwidth': '2'})


    g.view()

