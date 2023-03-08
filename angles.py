import matplotlib.pyplot as plt
import numpy as np
import point_2d


def plot_product_z_angle(n, save_dir):
    fig = plt.figure(figsize=(10, 10))
    plt.title("product_z_angle(i) [Radians]".upper())
    plt.xlim(0, n)
    plt.xlabel("time")
    plt.ylim(0, 1)
    plt.ylabel("angle [Rad]")
    t = [i for i in range(n)]
    th = [1/2] * n
    plt.plot(t, th, 'o')
    #plt.savefig(save_dir+"/product_z_angle.png")
    return fig


def plot_l1_l2_angle(n, save_dir):
    fig = plt.figure(figsize=(10, 10))
    plt.title("l1_l2_angle(i) [Radians]".upper())
    plt.xlim(0, n)
    plt.xlabel("time")
    plt.ylim(0, 1)
    plt.ylabel("angle [Rad]")
    t = [i for i in range(n)]
    th = [1/2] * n
    plt.plot(t, th, 'o')
    #plt.savefig(save_dir+"/l1_l2_angle.png")
    return fig


def plot_l2_l3_angle(n, save_dir):
    fig = plt.figure(figsize=(10, 10))
    plt.title("l2_l3_angle(i) [Radians]".upper())
    plt.xlim(0, n)
    plt.xlabel("time")
    plt.ylim(0, 1)
    plt.ylabel("angle [Rad]")
    t = [i for i in range(n)]
    th = [1/2] * n
    plt.plot(t, th, 'o')
    #plt.savefig(save_dir+"/l2_l3_angle.png")
    return fig


def plot_th_angle(x_points, y_points, save_dir):
    fig = plt.figure(figsize=(10, 10))
    plt.title("th_angle(i) [Radians]".upper())
    n = len(x_points)
    plt.xlim(0, n)
    plt.xlabel("time")
    plt.ylim(0, 1)
    plt.ylabel("angle [Rad]")
    t = [i for i in range(n)]
    th = [None]*n
    for i in range(n):
        th[i] = [point_2d.cartesian_to_polar(x_points[i], y_points[i])[1]/np.pi]
    plt.plot(t, th, 'o')
    #plt.savefig(save_dir+"/th_angle.png")
    return fig


def plot_angles(x_points, y_points):
    save_dir = "TEMP"
    n = len(x_points)
    plot_product_z_angle(n, save_dir)
    plot_l1_l2_angle(n, save_dir)
    plot_l2_l3_angle(n, save_dir)
    plot_th_angle(x_points, y_points, save_dir)


