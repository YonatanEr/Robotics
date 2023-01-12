import math
import numpy as np


def cartesian_to_polar(x, y):
    return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)


def polar_to_cartesian(r, th):
    return r * math.cos(th), r * math.sin(th)


def cartesian_to_polar_array(x, y):
    r, th = [], []
    for i in range(len(x)):
        pol = cartesian_to_polar(x[i], y[i])
        r.append(pol[0])
        th.append(pol[0])
    return np.array(r), np.array(th)


def polar_to_cartesian_array(r, th):
    x, y = [], []
    for i in range(len(r)):
        car = polar_to_cartesian(r[i], th[i])
        x.append(car[0])
        y.append(car[0])
    return np.array(x), np.array(y)
