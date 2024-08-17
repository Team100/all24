import time
import numpy as np

from matplotlib import pyplot as plt  # type:ignore

N = 200

rng = np.random.default_rng(0)

points = rng.uniform((0, 0), (5, 5), (N, 2))

fig = plt.figure()
axis = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
scatter = axis.scatter([], [])
fig.canvas.draw()

plt.show(block=False)

scatter.set_offsets(points)

axis.draw_artist(axis.patch)
axis.draw_artist(scatter)

# not sure why we need to do this twice
fig.canvas.flush_events()
fig.canvas.update()
fig.canvas.flush_events()

time.sleep(10)
