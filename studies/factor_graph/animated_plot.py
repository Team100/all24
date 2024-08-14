# pylint: disable=too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import numpy as np
from matplotlib import pyplot as plt  # type:ignore
from matplotlib.animation import FuncAnimation  # type:ignore

# initializing a figure in
# which the graph will be plotted
fig = plt.figure()

# marking the x-axis and y-axis
axis = plt.axes(xlim=(0, 4), ylim=(-2, 2))

# initializing a line variable
points = axis.scatter([], [])


# data which the line will
# contain (x, y)
def init():
    # line.set_data([], [])
    points.set_offsets(([], []))
    return (points,)


def animate(i):
    x = np.linspace(0, 4, 50)

    # plots a sine graph
    y = np.sin(2 * np.pi * (x - 0.01 * i))
    points.set_offsets(np.column_stack([x, y]))

    return (points,)


# NOTE: blit=True messes up the axes
anim = FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=False)

plt.show()
