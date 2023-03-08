import matplotlib.pyplot as plt
import Obstacle
import working_area
import plot_3d
import move
import numpy as np
import point_2d
import animate
import angles
import robot_kinematics_danny2

# The destination point
x_product = [-2, 0]

xd, yd, zd = x_product[0], x_product[1], 0.1
rd, thd = point_2d.cartesian_to_polar(xd, yd)[0], np.rad2deg(point_2d.cartesian_to_polar(xd, yd))[1]


def get_l(Q):
    tx, ty, tz = Q[0][-1], Q[1][-1], Q[2][-1]
    k = robot_kinematics_danny2.Inverse_Kinematics([tx, ty, tz])
    l = [k[1] + 1, k[0], 0.3 - k[2]]
    return l


def is_legal_point():
    if zd < 0.005:
        print("The point (", xd, yd, zd, ") collides with the ground")
        return False
    if Obstacle.obstacle_to_point_distance(xd, yd, zd) < 0.005:
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

    plt.title("The blue line is the ideal path, the green line is the controlled path")


    # start point
    x0, y0, z0 = plot_3d.plot_start_end_points(xd, yd, zd, ax)

    x_points, y_points, z_points = [x0], [y0], [z0]

    # l3 move up
    move.plot_move_l3(x_points=x_points, y_points=y_points, z_points=z_points, z1=0.25, n=2 ** 3, ax=ax, marker="^")
    Q1 = robot_kinematics_danny2.make_movement([1.5, 0, 0.1], [1.5, 0, 0.25])
    ax.scatter(Q1[0], Q1[1], Q1[2], color='green', s=3, marker='*')
    l1 = [x if x >= 0 else 360 + x for x in get_l(Q1)]
    # l1=get_l(Q1)

    # circular move
    move.plot_move_circle(x_points=x_points, y_points=y_points, z_points=z_points, xd=xd, yd=yd, n=2 ** 10, ax=ax)
    Q2 = robot_kinematics_danny2.make_movement(l1, [1.5, thd, 0.25], 2 ** 8)
    ax.scatter(Q2[0], Q2[1], Q2[2], color='green', s=10, marker='*')
    l2 = [x if x >= 0 else 360 + x for x in get_l(Q2)]

    # l2 = get_l(Q2)

    # l2 move
    move.plot_move_l2(x_points=x_points, y_points=y_points, z_points=z_points, x1=xd, y1=yd, n=2 ** 3, ax=ax)
    Q3 = robot_kinematics_danny2.make_movement(l2, [rd, thd, 0.25])
    ax.scatter(Q3[0], Q3[1], Q3[2], color='green', s=3, marker='*')
    l3 = [x if x >= 0 else 360 + x for x in get_l(Q3)]
    # l3 = get_l(Q3)

    # l3 move down
    move.plot_move_l3(x_points=x_points, y_points=y_points, z_points=z_points, z1=0.1, n=2 ** 3, ax=ax, marker="v")
    Q4 = robot_kinematics_danny2.make_movement(l3, [rd, thd, 0.1])
    ax.scatter(Q4[0], Q4[1], Q4[2], color='green', s=3, marker='*')

    x_con = Q1[0].tolist() + Q2[0].tolist() + Q3[0].tolist() + Q4[0].tolist()
    y_con = Q1[1].tolist() + Q2[1].tolist() + Q3[1].tolist() + Q4[1].tolist()
    z_con = Q1[2].tolist() + Q2[2].tolist() + Q3[2].tolist() + Q4[2].tolist()

    # Controlled path movement
    p1 = robot_kinematics_danny2.make_Q([1.5, 0, 0.1], [1.5, 0, 0.25])
    p2 = robot_kinematics_danny2.make_Q(l1, [1.5, thd, 0.25])
    p3 = robot_kinematics_danny2.make_Q(l2, [rd, thd, 0.25])
    p4 = robot_kinematics_danny2.make_Q(l3, [rd, thd, 0.1])
    robot_kinematics_danny2.plot_Q(p1[0], p1[1], p1[2])
    robot_kinematics_danny2.plot_Q(p2[0], p2[1], p2[2])
    robot_kinematics_danny2.plot_Q(p3[0], p3[1], p3[2])
    robot_kinematics_danny2.plot_Q(p4[0], p4[1], p4[2])



    return x_points, y_points, z_points, x_con, y_con, z_con


def run():
    if not is_legal_point():
        return False

    x_points, y_points, z_points, x_con, y_con, z_con = plot_path_plan()

    angles.plot_angles(x_points=x_points, y_points=y_points)

    animate.animate_movement(x_points=x_points, y_points=y_points, z_points=z_points, xd=xd, yd=yd, zd=zd,
                             x_con=x_con, y_con=y_con ,z_con=z_con)


run()
