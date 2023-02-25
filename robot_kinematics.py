import numpy as np

class Robot:

    def __init__(self):
        self.theta_min = -np.pi / 2
        self.theta_max = np.pi / 2
        self.h = 0.3  # m
        self.l_1 = 1  # m
        self.l_2_min = 0  # m
        self.l_2_max = 1  # m
        self.l_3_min = 0.05  # m
        self.l_3_max = 0.25  # m

    @staticmethod
    def Direct_Kinematics(theta, h, l_1, l_2, l_3):
        c = np.cos(theta)
        s = np.sin(theta)
        l = l_1 + l_2
        T = np.array([[-s, c, 0, l * c],
                      [c, s, 0, l + s],
                      [0, 0, -1, h - l_3],
                      [0, 0, 0, 1]])
        R = T[0:3, 0:3]
        P = T[0:3, 3]
        return T, R, P

    @staticmethod
    def Inverse_Kinematics(x, y, z, h, l_1):
        l_3 = h - z
        theta = np.arctan2(y, x)
        l_2 = (x / (np.cos(theta))) - l_1
        return theta, l_2, l_3

    @staticmethod
    def Jacobian(l_1, l_2, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        l = l_1 + l_2
        J_L = np.array([-l * s, c, 0],
                       [l * c, s, 0],
                       [0, 0, -1])

        J_A = np.array([0, 0, 0],
                       [0, 0, 0],
                       [1, 0, 0])
        return J_L, J_A
