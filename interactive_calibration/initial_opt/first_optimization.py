#!/usr/bin/env python
#
import matplotlib.pyplot as plt
import pylab as pl
import numpy as np
import math


def poly_cos(n, x):

    # This is a polynomial function that is more close to cos(x) as bigger is n
    # n is the polynomial degree (it only works till 10th )
    # x is the object
    # f is the image of x

    f = 0
    for i in range(0, n+1):
        if i % 2 == 0:
            f += x**(2*i) / math.factorial(2*i)
        elif i % 2 != 0:
            f -= x**(2*i) / math.factorial(2 * i)
    return f


def error(f, g):
    e = abs(f - g)
    return e


if __name__ == "__main__":
    pl.ion()
    x = np.arange(-2.5*np.pi, 2.5*np.pi, 0.1)
    x2 = np.arange(-2.5*np.pi, 2.5*np.pi, 1)
    fig = pl.plt.figure()
    ax = fig.add_subplot(211)
    ax.set_ylim(-1.5, 1.5)
    # pl.plt.xlabel("x")
    pl.plt.ylabel("y")
    pl.plt.title("function visualize")
    ax2 = fig.add_subplot(212)

    pl.plt.xlabel("evaluating points")
    pl.plt.ylabel("error")
    # pl.plt.title("x")
    myDict = {"cos": np.cos, "pol": poly_cos}
    for key in myDict:

        if key == "pol":
            for k in range(0, 11):
                ax.set_ylim(-1.5, 1.5)
                y = myDict[key](k, x)
                # ax = fig.add_subplot(211)
                pl.plt.plot(x, y, label=key)
                legend = ax.legend(loc='upper right', shadow=True, fontsize='x-large')

                y2 = error(np.cos(x2), poly_cos(k, x2))
                ax = fig.add_subplot(212)
                ax.set_ylim(-1, 70)
                pl.plt.xlabel("evaluating points")
                pl.plt.ylabel("error")
                pl.plt.plot(x2, y2, label=key)
                plt.pause(1)
                pl.clf()
                y = myDict['cos'](x)
                ax = fig.add_subplot(211)
                pl.plt.ylabel("y")
                pl.plt.title("function visualize")
                pl.plt.plot(x, y, label='cos')
                legend = ax.legend(loc='upper right', shadow=True, fontsize='x-large')

        elif key == "cos":
            y = myDict[key](x)
            ax = fig.add_subplot(211)
            pl.plt.plot(x, y, label=key)
            legend = ax.legend(loc='upper right', shadow=True, fontsize='x-large')
            plt.pause(3)

        # print myDict[key]
        # myDict.update({"cos": np.exp})

# exit(0)
