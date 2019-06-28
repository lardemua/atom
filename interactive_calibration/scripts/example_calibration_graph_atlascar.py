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

    cmap = cm.Pastel1(np.linspace(0, 1, 4 * 2))
    # cmap = cm.viridis(np.linspace(0, 1, len(xml_robot.sensors)))
    # cmap = cm.brg(np.linspace(0, 1, 4))
    # cmap = cm.Pastel1(np.linspace(0, 1, 4*2))

    i = 0;
    black = matplotlib.colors.rgb2hex([0, 0, 0])
    gray = matplotlib.colors.rgb2hex([0.4, 0.4, 0.4])

    # rgb1 = matplotlib.colors.rgb2hex(cmap[0, 0:3])
    # print(rgb1)
    # rgb2 = matplotlib.colors.rgb2hex(cmap[1, 0:3])
    # rgb3 = matplotlib.colors.rgb2hex(cmap[2, 0:3])
    rgb1 = 'darksalmon'
    rgb2 = 'lightsteelblue'
    rgb3 = 'darkseagreen'
    rgb4 = 'darkorchid2'

    g.edge('car_center', 'frontal_cam', '', color=rgb1, style='solid', _attributes={'penwidth': '1'})

    g.edge('car_center', 'top_right_cam', '', color=rgb2, style='solid', _attributes={'penwidth': '1'})

    g.edge('car_center', 'left_laser', '', color=rgb3, style='solid', _attributes={'penwidth': '1'})
    g.edge('car_center', 'right_laser', '', color=rgb4, style='solid', _attributes={'penwidth': '1'})

    g.edge('frontal_cam', 'frontal_cam_optical', '', color=rgb1, style='dashed', _attributes={'penwidth': '1'})
    g.edge('top_right_cam', 'top_right_cam_optical', '', color=rgb2, style='dashed',
           _attributes={'penwidth': '1'})


    # g.node('Reference Sensor', '<Reference Sensor \\n Parent Link - Sensor 1', _attributes={'penwidth': '1', 'color': black})
    # g.node('car_center', '< car_center<BR /> <FONT POINT-SIZE="12" COLOR="' + str(rgb1) + '">Sensor 1 parent</FONT> >', _attributes={'penwidth': '1', 'color': black})

    # text = '<car_center<BR /> ' + '<FONT POINT-SIZE="12" COLOR="' + str(
    #     rgb1) + '">parent: </FONT> <BR />' + ' <FONT POINT-SIZE="12" COLOR="' + str(
    #     rgb2) + '">top right cam parent</FONT> <BR />' + ' <FONT POINT-SIZE="12" COLOR="' + str(
    #     rgb3) + '">left laser parent</FONT> <BR />' + ' <FONT POINT-SIZE="12" COLOR="' + str(rgb4) + '">right_laser parent</FONT> >'

    text = '<car center<BR /> ' + '<FONT POINT-SIZE="12" COLOR="black">parent of: </FONT><FONT POINT-SIZE="12" COLOR="' + str(
        rgb1) + '"> frontal cam optical,<BR/></FONT> ' + ' <FONT POINT-SIZE="12" COLOR="' + str(
        rgb2) + '">top right cam optical,</FONT>' + ' <FONT POINT-SIZE="12" COLOR="' + str(
        rgb3) + '">left laser,</FONT>' + ' <FONT POINT-SIZE="12" COLOR="' + str(rgb4) + '">right laser</FONT> >'
    print(text)
    g.node('car_center', text,
           _attributes={'penwidth': '1', 'color': black})

    g.node('frontal_cam', '<frontal cam<BR/> <FONT POINT-SIZE="12" COLOR="' +
           str(rgb1) + '">frontal cam <BR/>optical child</FONT> >', _attributes={'penwidth': '1', 'color': black})
    g.node('top_right_cam', '<top right cam<BR /> <FONT POINT-SIZE="12" COLOR="' +
           str(rgb2) + '">top right cam <BR/>optical child</FONT> >', _attributes={'penwidth': '1', 'color': black})
    g.node('left_laser', '<left laser<BR /> <FONT POINT-SIZE="12" COLOR="' +
           str(rgb3) + '">left laser child</FONT> >', _attributes={'penwidth': '3', 'color': rgb3})
    g.node('right_laser', '<right laser<BR /> <FONT POINT-SIZE="12" COLOR="' +
           str(rgb4) + '">right laser child</FONT> >', _attributes={'penwidth': '3', 'color': rgb4})

    g.node('frontal_cam_optical', 'frontal cam \\n optical', _attributes={'penwidth': '3', 'color': rgb1})
    g.node('top_right_cam_optical', 'top right cam \\n optical', _attributes={'penwidth': '3', 'color': rgb2})


    # g.node('left_laser', 'left laser', _attributes={'penwidth': '3', 'color': rgb3})
    # g.node('right_laser', 'right laser', _attributes={'penwidth': '3', 'color': rgb4})


    g.view()
