import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from matplotlib import animation

##Robot geometry##
h=0.3 #m
l_1= 1 #m
l_2_min=0 #m
l_2_max=1 #m
l_3_min=0 #m
l_3_max=0.25 #m
m1=2#kg
m2=0.25#kg
m3=0.01#kg
I1=1
I2=1
I=I1+I2
g=-9.81
a = [l_1,h,m1,m2,m3,I,g]
fv = np.random.normal(0, 5, (3,))
fs = -np.random.normal(0, 5, (3,))
## q = (theta,l_2,l_3)


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
    if l_2<l_2_min or l_2>l_2_max:
        raise ValueError('l_2 is out of range')
    if l_3<l_3_min or l_3>l_3_max:
        raise ValueError('l_3 is out of range')
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
    #return [x,y,z]
    return [r*np.cos(np.deg2rad(theta)),r*np.sin(np.deg2rad(theta)),z]

def friction(q, dq):
    Ff = np.multiply(fv, dq) + np.multiply(fs, np.sign(dq))
    return Ff


def open_loop(qd, dqd, ddqd):
    u = M_matrix(qd,a).dot(ddqd) + C_matrix(qd, dqd,a).dot(dqd) + G_vector(qd,a)
    return u
def PD(q, dq, qd):
    Kp = np.diag([55, 40, 20])
    Kd = np.diag([25, 30, 10])
    e=q-qd[:3]
    G = G_vector(q,a)
    u = - Kp.dot(e) - Kd.dot(dq) + G
    return u

def computed_torque( q, dq, xg):
    Kp = np.diag([8000, 4000, 2000])
    Kd = np.diag([1500, 1000, 500])

    M = M_matrix(q, a)
    C = C_matrix(q, dq, a)
    G = G_vector(q, a)
    v = 0 - Kd.dot(dq - xg[3:]) - Kp.dot(q - xg[:3])
    u = M.dot( v ) + C.dot( dq ) + G
    return u

def model1(x, t, xg):
    # print(x)
    x1 = x[:3]
    x2 = x[3:]

    # Choose your controller here
    # u = open_loop(xg[:3], xg[3:], np.zeros((3,))).reshape((3,))
    u = PD(x1, x2, xg)
    # u = computed_torque(x1, x2, xg)

    invM = np.linalg.inv(M_matrix(x1,a))
    C = C_matrix(x1,x2,a)
    G = G_vector(x1,a)
    Ff =friction(x1,x2)*0
    dx1 = np.reshape(x2,(1,3))
    dx2 = np.reshape(invM.dot(np.reshape(u - C.dot(x2) - G - Ff,(3,1))),(1,3))
    dxdt = np.concatenate((dx1, dx2), axis = 1)
    return dxdt[0]

#set
k=Inverse_Kinematics(RToXyz(1.5,0,0.1),a)
b=Inverse_Kinematics(RToXyz(2,140,0.15),a)
x0 = np.array([np.deg2rad(k[0]),k[1],k[2], 0, 0, 0])
xg = np.array([np.deg2rad(b[0]),b[1],b[2], 0, 0, 0])
t = np.linspace(0, 5, 100)

Q = odeint(model1, x0, t, args=(xg,))

# Plot:
f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12,12))

#R
ax1.plot(t, np.rad2deg(Q[:,0]))
ax1.plot([0, np.max(t)], np.rad2deg([xg[0], xg[0]]), '--')
# ax1.plot([0, np.max(t)], np.rad2deg([xg[1], xg[1]]), '--')
ax1.set_title('Angles')
ax1.legend(('theta','theta target'))
ax1.set_xlabel('t (sec)')
ax1.set_ylabel('q (deg)')
ax1.set_xlim([0, np.max(t)])

ax2.plot(t, Q[:,3])
ax2.plot([0, np.max(t)], xg[4:], '--')
ax2.set_title('Angular velocity')
ax2.legend(('w_theta','w_theta target'))
ax2.set_xlabel('t (sec)')
ax2.set_ylabel('w (rad/sec)')
ax2.set_xlim([0, np.max(t)])

#PP
ax3.plot(t, Q[:,1:3])
ax3.plot([0, np.max(t)], [xg[1], xg[1]], '--')
ax3.plot([0, np.max(t)], [xg[2], xg[2]], '--')
ax3.set_title('Liner Movement')
ax3.legend(('l_2','l_3'))
ax3.set_xlabel('t (sec)')
ax3.set_ylabel('q (m)')
ax3.set_xlim([0, np.max(t)])

ax4.plot(t, Q[:,4:])
ax4.plot([0, np.max(t)], xg[4:], '--')
ax4.set_title('Linear velocity')
ax4.legend(('l_2','l_3'))
ax4.set_xlabel('t (sec)')
ax4.set_ylabel('v (m/sec)')
ax4.set_xlim([0, np.max(t)])
plt.show()
