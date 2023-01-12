import math
import matplotlib.pyplot as plt
import numpy as np
import point_2d

N = int(1e6)


def obstacle(n):
    radius = 0.2
    xc, yc, zc = 0, 1.5, 0
    height = 0.2
    r = radius * np.random.rand(n)
    th = 2 * math.pi * np.random.rand(n)
    x, y = point_2d.polar_to_cartesian_array(r, th)
    z = height * np.random.rand(n)
    x += xc
    y += yc
    z += zc
    return x, y, z


fig = plt.figure()
ax = fig.add_subplot()
xo, yo, zo = obstacle(N)
#ax.scatter3D(xo, yo, zo, color='r')


print(max((xo**2 + (yo-1.5)**2)))
print(min(xo**2 + (yo-1.5)**2))


plt.plot(xo, yo, color='r')

ax.set_xlim(-2, 2)
ax.set_ylim(0, 2)
#ax.set_zlim(0, 2)

ax.set_xlabel('x')
ax.set_ylabel('y')
#ax.set_zlabel('z')


plt.show()
