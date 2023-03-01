import numpy as np
import point_2d


def plot_move_circle(x0, y0, z0, xd, yd, n, ax):
    r0, th0 = point_2d.cartesian_to_polar(x0, y0)
    r1, th1 = point_2d.cartesian_to_polar(xd, yd)
    r = np.ones(n) * r0
    th = np.linspace(th0, th1, n)
    x, y = [], []
    for i in range(n):
        cart = point_2d.polar_to_cartesian(r[i], th[i])
        x.append(cart[0])
        y.append(cart[1])
    x = np.array(x)
    y = np.array(y)
    z = np.ones(n) * z0
    ax.scatter(x, y, z, color='blue', s=1)
    return x[-1], y[-1], z0


def plot_move_l2(x0, y0, z0, x1, y1, n, ax):
    # plots the working area
    r0, th0 = point_2d.cartesian_to_polar(x0, y0)
    r1, th1 = point_2d.cartesian_to_polar(x1, y1)
    r = np.linspace(r0, r1, n)
    th = np.ones(n) * th0
    x, y = [], []
    for i in range(n):
        cart = point_2d.polar_to_cartesian(r[i], th[i])
        x.append(cart[0])
        y.append(cart[1])
    x = np.array(x)
    y = np.array(y)
    z = np.ones(n) * z0
    ax.scatter(x, y, z, color='blue', s=4)
    return x1, y1, z0


def plot_move_l3(x0, y0, z0, z1, n, ax, marker="o"):
    # plots the working area
    x = np.ones(n) * x0
    y = np.ones(n) * y0
    z = np.linspace(z0, z1, n)
    ax.scatter(x, y, z, color='blue', s=4, marker=marker)
    return x0, y0, z1

