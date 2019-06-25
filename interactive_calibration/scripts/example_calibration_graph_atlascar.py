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

    cmap = cm.Pastel1(np.linspace(0, 1, 4*2))
    # cmap = cm.viridis(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.brg(np.linspace(0, 1, 4))
    # cmap = cm.Pastel1(np.linspace(0, 1, 4*2))


    i = 0;
    black = matplotlib.colors.rgb2hex([0, 0, 0])
    gray = matplotlib.colors.rgb2hex([0.4, 0.4, 0.4])

    # rgb1 = matplotlib.colors.rgb2hex(cmap[0, 0:3])
    # print(rgb1)
    rgb1 = 'darksalmon'
    # rgb2 = matplotlib.colors.rgb2hex(cmap[1, 0:3])
    rgb2 = 'lightsteelblue'
    # rgb3 = matplotlib.colors.rgb2hex(cmap[2, 0:3])
    rgb3 = 'darkseagreen'


    g.edge('car_center', 'frontal_camera', style='solid', _attributes={'penwidth': '1', 'color': rgb1 + ':' + rgb2})

    g.edge('car_center', 'Link B', '', color=rgb3, style='dashed', _attributes={'penwidth': '1'})

    g.edge('frontal_camera', 'Link C', '', color=rgb1, style='dashed', _attributes={'penwidth': '1'})
    g.edge('frontal_camera', 'Sensor 2', '', color=rgb2, style='solid', _attributes={'penwidth': '1'})

    g.edge('Link B', 'Sensor 3', '', color=rgb3, style='solid', _attributes={'penwidth': '1'})


    # g.node('Reference Sensor', '<Reference Sensor \\n Parent Link - Sensor 1', _attributes={'penwidth': '1', 'color': black})
    # g.node('car_center', '< car_center<BR /> <FONT POINT-SIZE="10" COLOR="' + str(rgb1) + '">Sensor 1 Calib Parent</FONT> >', _attributes={'penwidth': '1', 'color': black})

    g.node('car_center', '< car_center<BR /> <FONT POINT-SIZE="10" COLOR="' + str(rgb1) + '">Sensor 1 Calib Parent</FONT> >', _attributes={'penwidth': '1', 'color': black})
    g.node('frontal_camera', '<frontal_camera<BR /> <FONT POINT-SIZE="10" COLOR="' +
           str(rgb1) + '">Sensor 1 Calib Child</FONT> <BR /> <FONT POINT-SIZE="10" COLOR="' +
           str(rgb2) + '">Sensor 2 Calib Parent</FONT> >', _attributes={'penwidth': '1', 'color': black})

    g.node('Sensor 2', '<Sensor 2<BR /> <FONT POINT-SIZE="10" COLOR="' +
           str(rgb2) + '">Sensor 2 Calib Child</FONT> >', _attributes={'penwidth': '1', 'color': black})

    g.node('Link B', '<Link B<BR /> <FONT POINT-SIZE="10" COLOR="' +
           str(rgb3) + '">Sensor 3 Calib Parent</FONT> >', _attributes={'penwidth': '1', 'color': black})
    g.node('Sensor 3', '<Sensor 3<BR /> <FONT POINT-SIZE="10" COLOR="' +
           str(rgb3) + '">Sensor 3 Calib Parent</FONT> >', _attributes={'penwidth': '1', 'color': black})

    # g.edge('Sensor1', 'Sensor2', '',
    #        color=rgb, style='dashed', _attributes={'penwidth': '2'})
    # g.edge('Sensor2', 'Sensor3', '',
    #        color=rgb, style='dashed', _attributes={'penwidth': '2'})
    # g.edge('Sensor2', 'Sensor4', '',
    #        color=rgb, style='dashed', _attributes={'penwidth': '2'})


    g.view()

