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
m1=0.3#kg
m2=2#kg
m3=0.25#kg
g=9.81
a = [l_1,h,m1,m2,m3,g]
Fe = 5*np.array([np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand(),np.random.rand()])
fv = 5*np.array([np.random.rand(),np.random.rand(),np.random.rand()])
fs = -5*np.array([np.random.rand(),np.random.rand(),np.random.rand()])

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
def Inverse_Kinematics(loc):
    l_3 = np.around(h - loc[2],3)
    theta = np.arctan2(loc[1], loc[0])
    l_2 = np.around((loc[0] / (np.cos(theta))) - l_1,3)
    q= [np.rad2deg(theta), l_2, l_3]
    if theta<0 or theta>np.pi:
        print('ValueError : theta is out of working area setting back to the closest bound')
    #if l_2<l_2_min or l_2>l_2_max:
    #    print('ValueError : l_2 is out of working area')
    #if l_3<l_3_min or l_3>l_3_max:
    #    print('ValueError : l_3 is out of working area')
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
#a = [l_1,h,m1,m2,m3,g]
def M_matrix(q,a):
    M = np.zeros((len(q), len(q)))
    M[0, 0] = (((7/12)*a[3])+(2*a[4]))*((a[0]+q[1])**2)
    M[1, 1] = (a[3]/4)+a[4]
    M[2, 2] = a[4] / 4
    return M
def C_matrix(q,dq,a):
    C = np.zeros((len(q), len(q)))
    k = ((7*a[3]/12) + 2*a[4])*(a[0]+q[1])*dq[0]
    C[0, 1] = 2
    C[1, 0] = 1
    return C * k

def G_vector(q,a):
    G=np.array([0,0,0])
    G[2] = -a[4]*a[5]
    return G

def RToXyz(r,theta,z):
    #return [x,y,z]
    return [r*np.cos(np.deg2rad(theta)),r*np.sin(np.deg2rad(theta)),z]

def friction(q, dq):
    Ff = np.multiply(fv, dq) + np.multiply(fs, np.sign(dq))
    return Ff

def open_loop(qd, dqd, ddqd):
    u = M_matrix(qd,a).dot(ddqd) + C_matrix(qd, dqd, a).dot(dqd) + G_vector(qd, a)
    return u
def PD(q, dq, qd):
    Kp = np.diag([100, 80, 50])
    Kd = np.diag([50, 50, 50])
    e=q-qd[:3]
    G = G_vector(q,a)
    u = - Kp.dot(e) - Kd.dot(dq) + G
    return u

def computed_torque( q, dq, qd, dqd, ddqd):
    Kp = np.diag([2000, 1500, 3000])
    Kd = np.diag([200, 200, 100])

    M = M_matrix(q, a)
    C = C_matrix(q, dq, a)
    G = G_vector(q, a)
    v = ddqd - Kd.dot(dq - dqd) - Kp.dot(q - qd)
    u = M.dot( v ) + C.dot( dq ) + G
    return u

def traj_gen(q1, q2, t, Tf):
    a0 = q1
    a1 = np.zeros((3,))
    a3 = (q2 - q1) / (Tf**3 - 1.5*Tf**4)
    a2 = -1.5 * a3 * Tf**2

    q = a0 + a1*t + a2 * t**2 + a3 * t**3
    dq = a1 + 2*a2*t + 3*a3*t**2
    ddq = 2*a2 + 6*a3*t


    return q, dq, ddq
def model(x, t, x0, xg, Tf):
    # print(x)
    x1 = x[:3]
    x2 = x[3:]

    qd, dqd, ddqd = traj_gen(x0[:3], xg[:3], t, Tf)

    # Choose your controller here
    # u = open_loop(qd, dqd, ddqd)
    # u = PD(x1, x2, qd)
    u = computed_torque(x1, x2, qd, dqd, ddqd)

    invM = np.linalg.inv(M_matrix(x1,a))
    C = C_matrix(x1,x2,a)
    G = G_vector(x1,a)
    JtF = np.transpose(Jacobian(x1,a)).dot(Fe)
    Ff = friction(x1,x2)
    EF = Ff - JtF*0
    dx1 = np.reshape(x2,(1,3))
    dx2 = np.reshape(invM.dot(np.reshape(u - C.dot(x2) - G - EF,(3,1))),(1,3))
    dxdt = np.concatenate((dx1, dx2), axis = 1)
    # print(dxdt)
    return dxdt[0]

#set
def make_movement(loc_1,loc_2,n=2**3):
    h = 0.3  # m
    l_1 = 1  # m
    l_2_min = 0  # m
    l_2_max = 1  # m
    l_3_min = 0  # m
    l_3_max = 0.25  # m
    m1 = 0.3  # kg
    m2 = 2  # kg
    m3 = 0.25  # kg
    g = 9.81
    a = [l_1, h, m1, m2, m3, g]
    Fe = 0 * 5 * np.array(
        [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()])
    fv = 5 * np.array([np.random.rand(), np.random.rand(), np.random.rand()])
    fs = -5 * np.array([np.random.rand(), np.random.rand(), np.random.rand()])
    k = Inverse_Kinematics(RToXyz(loc_1[0], loc_1[1], loc_1[2]))
    b = Inverse_Kinematics(RToXyz(loc_2[0], loc_2[1], loc_2[2]))
    x0 = np.array([np.deg2rad(k[0]), k[1], k[2], 0, 0, 0])
    xg = np.array([np.deg2rad(b[0]), b[1], b[2], 0, 0, 0])
    Tf = 5
    t = np.linspace(0, Tf, n)

    Q = odeint(model, x0, t, args=(x0, xg, Tf,))[:,:3]
    R = Q[:,1]+l_1
    TH = [min(x,np.pi) if x>=0 else 0 for x in Q[:,0]]
    Z = 0.3-Q[:,2]
    X = R*np.cos(TH)
    Y = R*np.sin(TH)
    return X,Y,Z

def make_Q(loc_1,loc_2,n=2**3):
    h = 0.3  # m
    l_1 = 1  # m
    l_2_min = 0  # m
    l_2_max = 1  # m
    l_3_min = 0  # m
    l_3_max = 0.25  # m
    m1 = 0.3  # kg
    m2 = 2  # kg
    m3 = 0.25  # kg
    g = 9.81
    a = [l_1, h, m1, m2, m3, g]
    Fe = 0 * 5 * np.array(
        [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()])
    fv = 5 * np.array([np.random.rand(), np.random.rand(), np.random.rand()])
    fs = -5 * np.array([np.random.rand(), np.random.rand(), np.random.rand()])
    k = Inverse_Kinematics(RToXyz(loc_1[0], loc_1[1], loc_1[2]))
    b = Inverse_Kinematics(RToXyz(loc_2[0], loc_2[1], loc_2[2]))
    x0 = np.array([np.deg2rad(k[0]), k[1], k[2], 0, 0, 0])
    xg = np.array([np.deg2rad(b[0]), b[1], b[2], 0, 0, 0])
    Tf = 5
    t = np.linspace(0, Tf, n)

    Q = odeint(model, x0, t, args=(x0, xg, Tf,))
    return Q,x0,xg

k=Inverse_Kinematics(RToXyz(1.5,0,0.1))
b=Inverse_Kinematics(RToXyz(1.75,70,0.15))
x0 = np.array([np.deg2rad(k[0]),k[1],k[2], 0, 0, 0])
xg = np.array([np.deg2rad(b[0]),b[1],b[2], 0, 0, 0])
Tf = 5

Q=make_Q([1.5,0,0.1],[1.75,70,0.15],n=2**3)


def plot_Q(Q,x0,xg,n=2**3):
    Tf = 5
    t = np.linspace(0, Tf, n)
    Qd1 = np.array([traj_gen(x0[0], xg[0], tt, Tf)[0] for tt in t])
    Qd3a = np.array([traj_gen(x0[1], xg[1], tt, Tf)[0] for tt in t])
    Qd3b = np.array([traj_gen(x0[2], xg[2], tt, Tf)[0] for tt in t])

    dQd2 = np.array([traj_gen(x0[0], xg[0], tt, Tf)[1] for tt in t])
    dQd4a = np.array([traj_gen(x0[1], xg[1], tt, Tf)[1] for tt in t])
    dQd4b = np.array([traj_gen(x0[2], xg[2], tt, Tf)[1] for tt in t])

    # Plot:
    f, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12,12))

    #R
    ax1.plot(t, np.rad2deg(Q[:,0]))
    ax1.plot(t, np.rad2deg(Qd1)[:,0], '--')
    ax1.set_title('Angles')
    ax1.legend(('theta','theta target'))
    ax1.set_xlabel('t (sec)')
    ax1.set_ylabel('q (deg)')
    ax1.set_xlim([0, np.max(t)])


    ax2.plot(t, Q[:,3])
    ax2.plot(t, dQd2, '--')
    ax2.set_title('Angular velocity')
    ax2.legend(('w_theta','w_theta target'))
    ax2.set_xlabel('t (sec)')
    ax2.set_ylabel('w (rad/sec)')
    ax2.set_xlim([0, np.max(t)])


    #PP
    ax3.plot(t, Q[:,1:3])
    ax3.plot(t, Qd3a, '--')
    ax3.plot(t, Qd3b, '--')
    ax3.set_title('Liner Movement')
    ax3.legend(('l_2','l_3'))
    ax3.set_xlabel('t (sec)')
    ax3.set_ylabel('q (m)')
    ax3.set_xlim([0, np.max(t)])


    ax4.plot(t, Q[:,4:])
    ax4.plot(t, dQd4a, '--')
    ax4.plot(t, dQd4b, '--')
    ax4.set_title('Linear velocity')
    ax4.legend(('l_2','l_3'))
    ax4.set_xlabel('t (sec)')
    ax4.set_ylabel('v (m/sec)')
    ax4.set_xlim([0, np.max(t)])
    plt.show()


# plot_Q(Q[0],Q[1],Q[2])