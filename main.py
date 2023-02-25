import matplotlib.pyplot as plt
import obstacle
import working_area
import plot_3d
import move

# The destination point
x_product = [-2, 0]


xd, yd, zd = x_product[0], x_product[1], 0.1
n = 2**16


def is_legal_point():
    if zd < 0.005:
        print("The point (", xd, yd, zd, ") collides with the ground")
        return False
    if obstacle.obstacle_to_point_distance(xd, yd, zd) < 0.005:
        print("The point (", xd, yd, zd, ") collides with the obstacle")
        return False
    if not working_area.is_in_the_working_area(xd, yd, zd):
        print("The point (", xd, yd, zd, ") is not in the working area")
        return False
    return True


def run():
    if not is_legal_point():
        return False

    plt.figure(figsize=(10, 10))

    ax = plt.axes(projection='3d')
    ax.set_xlim(-2, 2)
    ax.set_ylim(0, 2)
    ax.set_zlim(0, 0.3)

    # start point
    x, y, z = plot_3d.plot_start_point(xd, yd, zd, n, ax)

    # l3 move up
    x, y, z = move.plot_move_l3(x0=x, y0=y, z0=z, z1=0.25, n=2**5, ax=ax, marker="^")

    # circular move
    x, y, z = move.plot_move_circle(x0=x, y0=y, z0=z, xd=xd, yd=yd, n=2**8, ax=ax)

    # l2 move
    x, y, z = move.plot_move_l2(x0=x, y0=y, z0=z, x1=xd, y1=yd, n=2**4, ax=ax)

    # l3 move down
    x, y, z = move.plot_move_l3(x0=x, y0=y, z0=z, z1=0.1, n=2**5, ax=ax, marker="v")

    plt.show()


run()


