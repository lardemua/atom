import math

import cv2
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


def drawSquare2D(image, x, y, size, color=(0, 0, 255), thickness=1):
    """
    Draws a square on the image
    :param image:
    :param x:
    :param y:
    :param color:
    :param thickness:
    """

    h, w, _ = image.shape
    if x - size < 0 or x + size >= w or y - size < 0 or y + size >= h:
        # print("Cannot draw square")
        return None

    # tl, tr, bl, br -> top left, top right, bottom left, bottom right
    tl = (int(x - size), int(y - size))
    tr = (int(x + size), int(y - size))
    br = (int(x + size), int(y + size))
    bl = (int(x - size), int(y + size))

    # cv2.line(image, (x,y), (x,y), color, 5)
    cv2.line(image, tl, tr, color, thickness)
    cv2.line(image, tr, br, color, thickness)
    cv2.line(image, br, bl, color, thickness)
    cv2.line(image, bl, tl, color, thickness)


def drawCross2D(image, x, y, size, color=(0, 0, 255), thickness=1):
    """
    Draws a square on the image
    :param image:
    :param x:
    :param y:
    :param color:
    :param thickness:
    """

    h, w, _ = image.shape
    if x - size < 0 or x + size > w or y - size < 0 or y + size > h:
        # print("Cannot draw square")
        return None

    # tl, tr, bl, br -> top left, top right, bottom left, bottom right
    left = (int(x - size), int(y))
    right = (int(x + size), int(y))
    top = (int(x), int(y - size))
    bottom = (int(x), int(y + size))

    cv2.line(image, left, right, color, thickness)
    cv2.line(image, top, bottom, color, thickness)


def drawTextOnImage(image, text,
                    font=cv2.FONT_HERSHEY_SIMPLEX,
                    position=(0, 0),
                    font_scale=3,
                    font_thickness=2,
                    text_color=(0, 255, 0),
                    text_color_bg=(0, 0, 0)):

    x, y = position
    x = int(x)
    y = int(y)
    margin = 10
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(image, (x-margin, y-margin), (x + text_w + margin, y + text_h + margin), text_color_bg, -1)
    cv2.rectangle(image, (x-margin, y-margin), (x + text_w + margin, y + text_h + margin), (0, 0, 0), 2)

    new_y = int(y + text_h + font_scale - 1)
    cv2.putText(image, text, (x, new_y), font, font_scale, text_color, font_thickness)
