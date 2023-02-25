import numpy as np
import matplotlib.pyplot as plt


def sticker_machine_collide(x, y, z):
    pass


def sticker_machine_to_point_distance(x, y, z):
    pass


def sticker_machine_coordinates(n):
    return 1.5, 0, 0.1


def plot_sticker_machine(n):
    plt.figure()
    ax = plt.axes(projection='3d')
    x, y, z = sticker_machine_coordinates(n)
    ax.plot3D(x, y, z, 'blue')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-0, 2)
    ax.set_zlim(-0, 0.2)
    plt.show()
