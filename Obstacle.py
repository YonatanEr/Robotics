import numpy as np
import point_2d
import scipy
import matplotlib


def obstacle_collide(x, y, z):
    return (0 <= z) and (z <= 0.2) and (x ** 2 + (y - 1.5) ** 2 <= 0.2 ** 2)


def obstacle_to_point_distance(x, y, z):
    # a = x,y diff from obstacle
    # b = z diff from obstacle
    a = max(0, (x ** 2 + (y - 1.5) ** 2) ** 0.5 - 0.2)
    b = 0
    if z < 0:
        b = z
    if 0.2 < z:
        b = z - 0.2
    return (a ** 2 + b ** 2) ** 0.5


def obstacle_coordinates(n):
    r = 0.2 * np.random.rand(n)
    th = 2 * np.pi * np.random.rand(n)
    x, y = point_2d.polar_to_cartesian_array(r, th)
    z = 0.2 * np.random.rand(n)
    return x, y, z

