import numpy as np
import point_2d


def plot_move_circle(x_points, y_points, z_points, xd, yd, n, ax):
    x0, y0, z0 = x_points[-1], y_points[-1], z_points[-1]
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
    x_points += x.tolist()
    y_points += y.tolist()
    z_points += z.tolist()


def plot_move_l2(x_points, y_points, z_points, x1, y1, n, ax):
    x0, y0, z0 = x_points[-1], y_points[-1], z_points[-1]
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
    x_points += x.tolist()
    y_points += y.tolist()
    z_points += z.tolist()


def plot_move_l3(x_points, y_points, z_points, z1, n, ax, marker="o"):
    x0, y0, z0 = x_points[-1], y_points[-1], z_points[-1]
    x = np.ones(n) * x0
    y = np.ones(n) * y0
    z = np.linspace(z0, z1, n)
    ax.scatter(x, y, z, color='blue', s=8, marker=marker)
    x_points += x.tolist()
    y_points += y.tolist()
    z_points += z.tolist()

