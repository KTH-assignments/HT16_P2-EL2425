import trajectory_planner
import matplotlib.pyplot as plt
import numpy as np

path = trajectory_planner.Path()
circ = path.get_points()

xs = []
ys = []
ts = []
for point in circ:
    xs.append(point[0])
    ys.append(point[1])
    ts.append(point[2])

plt.plot(xs, ys)
plt.axis('equal')
plt.show()

plt.plot(ts)
plt.show()
