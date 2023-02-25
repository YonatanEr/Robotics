import numpy as np
from matplotlib import pyplot as plt
import point_2d


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


def plot_obstacle(n):
    plt.figure()
    ax = plt.axes(projection='3d')
    x, y, z = working_area_coordinates(n)
    ax.scatter(x, y, z, color='gray')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-0, 2)
    ax.set_zlim(-0, 0.2)
    plt.show()
