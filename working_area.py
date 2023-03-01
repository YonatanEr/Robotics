import numpy as np
import point_2d


def is_in_the_working_area(x, y, z):
    if y < 0 or z < 0 or (x ** 2 + y ** 2) < 1 or 4 < (x ** 2 + y ** 2):
        return False
    return True


def working_area_coordinates(n):
    x, y = [], []
    r = np.random.rand(n) + 1
    th = np.pi * np.random.rand(n)
    for i in range(n):
        cart = point_2d.polar_to_cartesian(r[i], th[i])
        x.append(cart[0])
        y.append(cart[1])
    x = np.array(x)
    y = np.array(y)
    z = np.zeros(n)
    return x, y, z
