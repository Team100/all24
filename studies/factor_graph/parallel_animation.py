# pylint: disable=missing-function-docstring,missing-module-docstring

# illustrates animation without blocking on the animation itself.
# instead the main loop updates the animation periodically.

import time
import math
import numpy as np
from matplotlib import pyplot as plt  # type:ignore
from matplotlib.animation import FuncAnimation  # type:ignore

N = 200


def main():
    rng = np.random.default_rng(0)
    points = rng.uniform((0, 0), (5, 5), (N, 2))
    fig = plt.figure()
    axis = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
    scatter = axis.scatter([], [], marker="o", s=10)

    def animate(_):
        return (scatter,)

    _ = FuncAnimation(fig, animate, interval=0, blit=True)

    plt.show(block=False)
    t0 = time.time()
    max_time_sec = 5
    while (d := time.time() - t0) < max_time_sec:
        points += np.ones_like(points) * math.sin(math.pi * d) / 100
        scatter.set_offsets(points)
        fig.canvas.flush_events()  # blocks for a few ms


if __name__ == "__main__":
    main()
