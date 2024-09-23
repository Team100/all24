import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter
from matplotlib.patches import Rectangle
from scenario import Scenario


def plot_linkage(sc: Scenario, ax, P1, P2, P3, P4, P5, alpha=1):
    """Plots the linkage in the specified position."""

    x1 = P1[0].item()
    y1 = P1[1].item()
    x2 = P2[0].item()
    y2 = P2[1].item()
    x3 = P3[0].item()
    y3 = P3[1].item()
    x4 = P4[0].item()
    y4 = P4[1].item()
    x5 = P5[0].item()
    y5 = P5[1].item()
    ax.axis("equal")
    ax.set_xlim(sc.xmin, sc.xmax)
    ax.set_ylim(sc.ymin, sc.ymax)
    ax.grid()
    ax.add_patch(Rectangle((sc.left(), sc.bottom()), sc.w, sc.h, fill=False))
    ax.plot(x1, y1, marker="o", alpha=alpha)
    ax.plot(x2, y2, marker="o", alpha=alpha)
    ax.plot(x3, y3, marker="o")  # the important one :-)
    ax.plot(x4, y4, marker="o", alpha=alpha)
    ax.plot(x5, y5, marker="o", alpha=alpha)
    ax.plot([x1, x2], [x1, y2], alpha=alpha)
    ax.plot([x2, x3], [y2, y3], alpha=alpha)
    ax.plot([x3, x4], [y3, y4], alpha=alpha)
    ax.plot([x4, x5], [y4, y5], alpha=alpha)
    ax.plot([x5, x1], [y5, y1], alpha=alpha)


def plot_contour(name: str, sc: Scenario, xpoints, ypoints, zpoints):
    """Plots the z data as a color map and labels the contours.

    Returns:
        the figure, so the caller can save it
    """
    fig = plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.axis("equal")
    ax.set_xlim(sc.xmin, sc.xmax)
    ax.set_ylim(sc.ymin, sc.ymax)
    ax.grid()
    ax.add_patch(Rectangle((sc.left(), sc.bottom()), sc.w, sc.h, fill=False))
    smoothed = gaussian_filter(zpoints, 1)
    ax.contourf(xpoints, ypoints, smoothed, cmap="summer")
    CS = ax.contour(xpoints, ypoints, smoothed, colors="k")
    ax.clabel(CS)
    ax.set_title(name)
    return fig
