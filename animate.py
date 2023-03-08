import matplotlib
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


def update(num, data, data1, line, line1):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])
    line1.set_data(data1[:2, :num])
    line1.set_3d_properties(data1[2, :num])


def animate_movement(x_points, y_points, z_points, xd, yd, zd , x_con , y_con , z_con):
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
    x_points, y_points, z_points = x_points[::16] + x_points[-4:-1], \
                                   y_points[::16] + y_points[-4:-1], \
                                   z_points[::16] + z_points[-4:-1]

    N = min(len(x_points), len(x_con))

    x_con , y_con , z_con = x_con[::4] + x_con[-2:-1],\
                            y_con[::4] + y_con[-2:-1],\
                            z_con[::4] + z_con[-2:-1],\

    data = np.array(list(gen(x_points, y_points, z_points))).T
    line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1], color="blue")
    

    data1 = np.array(list(gen(x_con , y_con , z_con))).T
    line1, = ax.plot(data1[0, 0:1], data1[1, 0:1], data1[2, 0:1], color="green")


    # I'm assuming you will run the animation using 60 FPS monitor
    ani = animation.FuncAnimation(fig, update, N, fargs=(data, data1, line, line1), interval=1/60*1e3)

    plt.title("The blue line is the ideal path, the green line is the controlled path")
    plt.show()
