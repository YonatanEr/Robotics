import matplotlib.pyplot as plt
import obstacle
import working_area
import plot_3d
import move
import animate
import angles

# The destination point
x_product = [-2, 0]

xd, yd, zd = x_product[0], x_product[1], 0.1


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


def plot_path_plan():
    plt.figure(figsize=(10, 10))
    ax = plt.axes(projection='3d')
    ax.set_xlim(-2, 2)
    ax.set_xlabel('x')
    ax.set_ylim(0, 2)
    ax.set_ylabel('y')
    ax.set_zlim(0, 0.3)
    ax.set_zlabel('z')

    # start point
    x0, y0, z0 = plot_3d.plot_start_end_points(xd, yd, zd, ax)

    x_points, y_points, z_points = [x0], [y0], [z0]

    # l3 move up
    move.plot_move_l3(x_points=x_points, y_points=y_points, z_points=z_points, z1=0.25, n=2**3, ax=ax, marker="^")


    # circular move
    move.plot_move_circle(x_points=x_points, y_points=y_points, z_points=z_points, xd=xd, yd=yd, n=2**8, ax=ax)


    # l2 move
    move.plot_move_l2(x_points=x_points, y_points=y_points, z_points=z_points, x1=xd, y1=yd, n=2**3, ax=ax)


    # l3 move down
    move.plot_move_l3(x_points=x_points, y_points=y_points, z_points=z_points, z1=0.1, n=2**3, ax=ax, marker="v")

    #plt.savefig("outputs/3/path_plan.png")
    plt.show()

    return x_points, y_points, z_points



def run():
    if not is_legal_point():
        return False


    x_points, y_points, z_points = plot_path_plan()

    angles.plot_angles(x_points=x_points, y_points=y_points)

    animate.animate_movement(x_points=x_points, y_points=y_points, z_points=z_points, xd=xd, yd=yd, zd=zd)


run()


