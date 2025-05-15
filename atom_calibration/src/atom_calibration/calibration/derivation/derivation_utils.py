#!/usr/bin/env python3

from typing import Dict, List


def centralNumericalFirstDerivative(data: Dict[str, float]) -> List[float]:
    """
    Numerically calculates the first derivative at instant t_i using central derivation.

    Input:
        data: a dictionary of the relevant data for derivation. The dictionary must have the following structure:

        data = {
            "t": [
                t_{i-1},
                t_i,
                t_{i+1},
                ],
            "var1": [
                var1(t_{i-1}),
                var(t_i),
                var(t_{i+1}),
                ],
            ....
            "varN": [
                varN(t_{i-1}),
                varN(t_i),
                varN(t_{i+1}),
                ]
            }

    Output:
        ddata_dt: a list with the derivatives at instant t, in order of the variables of the input dictionary

        ddata_dt = [
            dvar1_dt(t_i),
            ...,
            dvarN_dt(t_i)
            ]
    """
    ddata_dt = []

    for var in data.keys():
        if var == "t":
            continue

        dvar_dt = (data[var][2] - data[var][0]) / (data["t"][2] - data["t"][0])
        ddata_dt.append(dvar_dt)

    return ddata_dt


def centralNumericalSecondDerivative(data: Dict[str, float]) -> List[float]:
    """
    Numerically calculates the second derivative at instant t_i using central derivation.

    Input:
        data: a dictionary of the relevant data for derivation. Uses 3 datapoints, where t_i is the central instant. The dictionary must have the following structure:

        data = {
            "t": [
                t_{i-1},
                t_i,
                t_{i+1},
                ],
            "var1": [
                var1(t_{i-1}),
                var(t_i),
                var(t_{i+1}),
                ],
            ....
            "varN": [
                varN(t_{i-1}),
                varN(t_i),
                varN(t_{i+1}),
                ]
            }

    Output:
        dddata_dtt: a list with the derivatives at instant t, in order of the variables of the input dictionary

        dddata_dtt = [
            ddvar1_dtt(t_i),
            ...,
            ddvarN_dtt(t_i)
            ]
    """
    dddata_dtt = []

    for var in data.keys():
        if var == "t":
            continue

        ddvar_dtt = (data[var][2] - 2 * data[var][1] + data[var][0]) / (
            (data["t"][2] - data["t"][1]) ** 2
        )
        dddata_dtt.append(ddvar_dtt)

    return dddata_dtt
