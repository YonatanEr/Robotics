import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

##Robot geometry##
h=0.3 #m
l_1= 1 #m
l_2_min=0 #m
l_2_max=1 #m
l_3_min=0 #m
l_3_max=0.25 #m
m1=1
m2=1
m3=0
I1=1
I2=1
I=I1+I2
g=9.81
a = [l_1,h,m1,m2,m3,I,g]


    ##Robot kinamatics##
def Direct_Kinematics(q):
    c = np.cos(q[0])
    s = np.sin(q[0])
    l = l_1 + q[1]
    T = np.array([[-s, c, 0, l * c],
                  [c, s, 0, l + s],
                  [0, 0, -1, h - q[2]],
                  [0, 0, 0, 1]])
    R = T[0:3, 0:3]
    P = T[0:3, 3]
    return T, R, P
def Inverse_Kinematics(loc, a):
    l_3 = np.around(a[1] - loc[2],3)
    theta = np.arctan2(loc[1], loc[0])
    l_2 = np.around((loc[0] / (np.cos(theta))) - a[0],3)
    q= [np.rad2deg(theta), l_2, l_3]
    # if l_2<l_2_min or l_2>l_2_max:
    #     raise ValueError('l_2 is out of range')
    # if l_3<l_3_min or l_3>l_3_max:
    #     raise ValueError('l_3 is out of range')
    return q
def Jacobian(q,a):
    c = np.cos(q[0])
    s = np.sin(q[0])
    l = a[0] + q[1]
    J_L = np.array([[-l * s, c, 0],
                   [l * c, s, 0],
                   [0, 0, -1]])

    J_A = np.array([[0, 0, 0],
                   [0, 0, 0],
                   [1, 0, 0]])
    return np.concatenate((J_L, J_A), axis=0)
def M_matrix(q,a):
    M = np.zeros((len(q), len(q)))
    W = (a[2] / 4) + a[3] + a[4]
    M[0, 0] = ((a[0] + q[1]) ** 2) * (W) + a[5]
    M[1, 1] = W
    M[2, 2] = a[4] + a[3] / 4
    return M
def C_matrix(q,dq,a):
    C = np.zeros((len(q), len(q)))
    W = (a[2] / 4) + a[3] + a[4]
    C[0, 1] = 2
    C[1, 0] = -1
    return C * dq[0] * (q[1] + a[0]) * W
def G_vector(q,a):
    G=np.array([0,0,0])
    # G = np.zeros((1,len(q)))
    W = a[6] * (a[4] + a[3] / 2)
    G[2] = 1
    return G * W
def RToXyz(r,theta,z):
    return [r*np.cos(np.deg2rad(theta)),r*np.sin(np.deg2rad(theta)),z]

# def friction(q, dq):
#     fv = np.random.normal(0, 0.01, (3,))
#     fs = -fv
#     Ff = np.multiply(fv, dq) + np.multiply(fs, np.sign(dq))
#     return Ff

def traj_gen(q1, q2, t, Tf):
    a0 = q1
    a1 = np.zeros((2,))
    a2 = -1.5 * a3 * Tf**2
    a3 = (q2 - q1) / (Tf**3 - 1.5*Tf**4)


    q = a0 + a1*t + a2 * t**2 + a3 * t**3
    dq = a1 + 2*a2*t + 3*a3*t**2
    ddq = 2*a2 + 6*a3*t

    return q, dq, ddq

def open_loop(qd, dqd, ddqd):
    u = M_matrix(qd,a).dot(ddqd) + C_matrix(qd, dqd,a).dot(dqd) + G_vector(qd,a)
    return u
def PD(q, dq, qd):
    Kp = np.diag([1500, 1000, 500])
    Kd = np.diag([30, 50, 70])
    # print(f'q={q}')
    # print(f'dq={dq}')
    # print(f'qd={qd}')
    # print(f'Kp={Kp}')
    # print(f'Kd={Kd}')
    u = - Kp.dot(q-qd) - Kd.dot(dq)

    return u
def model(x, t, xg):
    x1 = x[:3]
    x2 = x[3:]
    # print(x)
    # Choose your controller here
    # u = open_loop(xg[:3], xg[3:], np.zeros((3,))).reshape((3,))
    u = PD(x1, x2, xg).reshape((3,))
    #u = computed_torque(x1, x2, xg)
    # u=np.zeros((1,3))

    invM = np.linalg.inv(M_matrix(x1,a))
    C = C_matrix(x1,x2,a)
    G = G_vector(x1,a)
    Ff = np.zeros((1,3))
    # print(invM.dot(C.dot(x2)))
    dx1 = x2
    dx2 = invM.dot(invM.dot(C.dot(x2))- G)
    # dx2 = invM.dot(u - C.dot(x2) - np.transpose(G) - Ff)
    # print(dx1)
    # print(f'dx2={invM.dot(C.dot(x2))- G}')

    dxdt = np.concatenate((dx1, dx2), axis = 0)

    return dxdt


# theta=45
# r=1.1
# z=0.3
#
# x=np.linspace(1.8,1.4,100)
# y=np.linspace(1.2,1.7,100)
# z=np.linspace(0.1,0.2,100)
# q=Inverse_Kinematics(RToXyz(r,theta,z),a)
# move=Inverse_Kinematics([x,y,z],a)[0]
# # plt.plot(x,move)
# # plt.show()
# M=M_matrix(q,a)
# C=C_matrix(q,q,a)
# G=G_vector(q,a)
# Ff = friction(q, q)
# print (G)
# print(Jacobian(q,a))

x0 = np.array([0.7, -0.5, 0, 1, 2, 2])
xg = np.array([0.3, 0.4, 0])
t = np.linspace(0, 3, 1000)

Q = odeint(model, x0, t, args=(xg,))
print(Q)
X = np.dtype(float)
# X = np.array([Direct_Kinematics(q) for q in Q])
# for q in Q:
#     print(Direct_Kinematics(q))

# Plot:
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(12,5))
ax1.plot(t, np.rad2deg(Q[:,:3]))
ax1.plot([0, np.max(t)], np.rad2deg([xg[0], xg[0]]), '--')
ax1.plot([0, np.max(t)], np.rad2deg([xg[1], xg[1]]), '--')
ax1.set_title('Angles')
ax1.legend(('q1','q2','q3'))
ax1.set_xlabel('t (sec)')
ax1.set_ylabel('q (deg)')
ax1.set_xlim([0, np.max(t)])

# ax2.plot(t, Q[:,2:])
# ax2.plot([0, np.max(t)], xg[2:], '--')
# ax2.set_title('Angular velocity')
# ax2.legend(('w1','w2'))
# ax2.set_xlabel('t (sec)')
# ax2.set_ylabel('w (rad/sec)')
# ax2.set_xlim([0, np.max(t)])
# plt.show()
