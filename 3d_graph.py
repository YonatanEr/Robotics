import matplotlib.pyplot as plt
import Obstacle
import StickerMachine
import WorkingArea

n = int(1e4)

plt.figure()
ax = plt.axes(projection='3d')
x, y, z = Obstacle.obstacle_coordinates(n)
ax.plot3D(x, y, z, 'red')
x, y, z = StickerMachine.sticker_machine_coordinates(n)
ax.plot3D(x, y, z, 'blue')
x, y, z = WorkingArea.working_area_coordinates(n)
ax.scatter(x, y, z, color='gray', s=1)
ax.set_xlim(-2, 2)
ax.set_ylim(-0, 2)
ax.set_zlim(-0, 0.2)
plt.show()

