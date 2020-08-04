import math
import numpy as np


def draw_concentric_circles(plt, radii, color='tab:gray'):
    for r in radii:  # meters
        x = []
        y = []
        for theta in np.linspace(0, 2 * math.pi, num=150):
            x.append(r * math.cos(theta))
            y.append(r * math.sin(theta))
        plt.plot(x, y, '-', color='tab:gray')

        x = r * math.cos(math.pi / 2)
        y = r * math.sin(math.pi / 2)
        plt.text(x + .5, y, str(r) + ' (m)', color='tab:gray')


def draw_2d_axes(plt):
    plt.ylabel('y')
    plt.xlabel('x')
    ax = plt.axes()
    ax.arrow(0, 0, 10, 0, head_width=.5, head_length=1, fc='r', ec='r')
    ax.arrow(0, 0, 0, 10, head_width=.5, head_length=1, fc='g', ec='g')

    plt.text(10, 0.5, 'X', color='red')
    plt.text(-1.0, 10, 'Y', color='green')


def colormapToRVizColor(color):
    """ Converts a Matbplotlib colormap into an rviz display color format."""
    return str(int(color[0] * 255)) + '; ' + str(int(color[1] * 255)) + '; ' + str(
        int(color[2] * 255))