import Obstacle
import sticker_machine
import working_area


def plot_start_end_points(xd, yd, zd, ax, n = 2**16):
    # plots the working area
    x, y, z = working_area.working_area_coordinates(n)
    ax.scatter(x, y, z, color='Gray', s=1, alpha=0.05)

    # plots the obstacle
    x, y, z = Obstacle.obstacle_coordinates(n)
    ax.scatter(x, y, z, color='Red', s=100, alpha=0.01)

    # plots the sticker machine
    x, y, z = sticker_machine.sticker_machine_coordinates()
    ax.scatter(x, y, z, 'Blue', s=100, marker='^', alpha=1)
    ax.text(x, y ,z, '  ({}, {}, {})    '.format(x, y, z))

    # plots the destination point
    ax.scatter(xd, yd, zd, 'Orange', s=100, marker='v', alpha=1)
    ax.text(xd, yd ,zd, '   ({}, {}, {})    '.format(xd, yd, zd))

    return x, y, z
