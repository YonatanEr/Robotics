import obstacle
import sticker_machine
import working_area


def plot_start_point(xd, yd, zd, n, ax):
    # plots the working area
    x, y, z = working_area.working_area_coordinates(n)
    ax.scatter(x, y, z, color='Gray', s=1)

    # plots the obstacle
    x, y, z = obstacle.obstacle_coordinates(n)
    ax.scatter(x, y, z, color='Red', s=10)

    # plots the sticker machine
    x, y, z = sticker_machine.sticker_machine_coordinates()
    ax.scatter(x, y, z, 'Blue', s=100, marker='^')

    # plots the destination point
    ax.scatter(xd, yd, zd, 'Orange', s=100, marker='v')

    return x, y, z
