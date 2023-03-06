from matplotlib import pyplot as plt
import numpy as np
from matplotlib import animation

from plot_3d import plot_start_end_points


def gen(x_points, y_points, z_points):
    # returns a generator which iterates over the input lists tuples (x, y ,z)
    i = 0
    while i < len(x_points):
        yield np.array([x_points[i], y_points[i], z_points[i]])
        i += 1


def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])


def animate_movement(x_points, y_points, z_points, xd, yd, zd):
    fig = plt.figure(figsize=(10, 10))
    ax = plt.axes(projection='3d')
    ax.set_xlim(-2, 2)
    ax.set_xlabel('x')
    ax.set_ylim(0, 2)
    ax.set_ylabel('y')
    ax.set_zlim(0, 0.3)
    ax.set_zlabel('z')

    plot_start_end_points(xd, yd, zd, ax)

    # To many points causes slowness
    x_points, y_points, z_points = x_points[::8]+[x_points[-1]], \
                                   y_points[::8]+[y_points[-1]], \
                                   z_points[::8]+[z_points[-1]]

    N = len(x_points)
    data = np.array(list(gen(x_points, y_points, z_points))).T
    line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])


    # I'm assuming you will run the animation using 60 FPS monitor
    ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=1/60*1e3)

    #ani.save("outputs/3/path_simulation.gif", animation.PillowWriter(fps=60))
    plt.show()

