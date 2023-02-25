import math
import numpy as np


def cartesian_to_polar(x, y):
    return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)


def polar_to_cartesian(r, th):
    return r * math.cos(th), r * math.sin(th)

