# pylint: disable=missing-function-docstring,missing-module-docstring

# illustrates animation without FuncAnimation.

import time
import math
import numpy as np
from matplotlib import pyplot as plt  # type:ignore

N = 200


def main():
    rng = np.random.default_rng(0)
    points = rng.uniform((0, 0), (5, 5), (N, 2))
    fig = plt.figure()
    axis = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
    scatter = axis.scatter([], [], marker="o", s=10)
    fig.canvas.draw()

    plt.show(block=False)
    t0 = time.time()
    max_time_sec = 5
    while (d := time.time() - t0) < max_time_sec:
        points += np.ones_like(points) * math.sin(math.pi * d) / 100
        scatter.set_offsets(points)
        # just draw the data we want to update
        # this is much faster than draw_idle
        axis.draw_artist(axis.patch)
        axis.draw_artist(scatter)
        # skip redrawing the axes since it's very slow to do so
        fig.canvas.update()
        fig.canvas.flush_events()


if __name__ == "__main__":
    main()
