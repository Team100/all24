import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import kinematics
from plot import plot_linkage, plot_contour
from scenario import Scenario


def reach(sc: Scenario):
    """Verifies the end-effector can reach the whole envelope.

    Simulates the linkage moving around, checks that the kinematics
    calculations succeed.
    """

    xpoints = np.linspace(sc.right(), sc.left(), 20)
    ypoints = np.linspace(sc.top(), sc.bottom(), 10)
    fig = plt.figure(figsize=(8, 8))
    ax = plt.gca()
    for ix, x3 in enumerate(xpoints):
        for iy, y3 in enumerate(ypoints):
            P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, x3, y3)
            plot_linkage(sc, ax, P1, P2, P3, P4, P5, 0.1)
    ax.set_title("Reachability")
    fig.savefig('reach')


def examples(sc):
    """Plot the key positions."""
    fig, axs = plt.subplots(3, 3, figsize=(8, 8))
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.left(), sc.top())
    plot_linkage(sc, axs[0, 0], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.xcenter, sc.top())
    plot_linkage(sc, axs[0, 1], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.right(), sc.top())
    plot_linkage(sc, axs[0, 2], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.left(), sc.ycenter)
    plot_linkage(sc, axs[1, 0], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.xcenter, sc.ycenter)
    plot_linkage(sc, axs[1, 1], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.right(), sc.ycenter)
    plot_linkage(sc, axs[1, 2], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.left(), sc.bottom())
    plot_linkage(sc, axs[2, 0], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.xcenter, sc.bottom())
    plot_linkage(sc, axs[2, 1], P1, P2, P3, P4, P5)
    P1, P2, P3, P4, P5, Ph = kinematics.joints(sc, sc.right(), sc.bottom())
    plot_linkage(sc, axs[2, 2], P1, P2, P3, P4, P5)
    axs[0,1].set_title("Examples")
    fig.savefig('examples')


def envelope(sc):
    t1points = np.linspace(-np.pi / 4, 5 * np.pi / 8, 20)
    t5points = np.linspace(3 * np.pi / 8, 5 * np.pi / 4, 20)
    fig = plt.figure(figsize=(8, 8))
    ax = plt.gca()
    for it1, t1 in enumerate(t1points):
        for it5, t5 in enumerate(t5points):
            P1, P2, P3, P4, P5, Ph = kinematics.forward(sc, t1, t5)
            if P3[1] < 0:
                continue
            plot_linkage(sc, ax, P1, P2, P3, P4, P5, 0.1)
    ax.set_title("Full Envelope")
    fig.savefig('envelope')


def interior(sc):
    """Plots the Jacobian condition number and minimum-maximum force over the workspace."""
    xpoints = np.linspace(sc.right(), sc.left(), 20)
    ypoints = np.linspace(sc.top(), sc.bottom(), 10)
    condition = np.zeros([len(ypoints), len(xpoints)])
    min_force = np.zeros([len(ypoints), len(xpoints)])
    for ix, x3 in enumerate(xpoints):
        for iy, y3 in enumerate(ypoints):
            t1, t5 = kinematics.inverse(sc, x3, y3)
            P1, P2, P3, P4, P5, Ph = kinematics.forward(sc, t1, t5)
            J = kinematics.jacobian(sc, t1, t5, P1, P2, P3, P4, P5, Ph)
            condition[iy, ix] = np.linalg.cond(J)  # [y,x] = row major
            min_force[iy, ix] = min_max_force(sc, J)  # [y,x] = row major
    fig = plot_contour("Condition Number", sc, xpoints, ypoints, condition)
    fig.savefig('condition')
    fig = plot_contour("Min Force (N)", sc, xpoints, ypoints, min_force)
    fig.savefig('force')


def min_max_force(sc, J):
    """The minimum force over the circle corresponding to motor stall."""
    grain = 12
    angles = np.linspace(0, 2 * np.pi, grain)
    max_dir_force = float("inf")
    for angle in angles:
        max_dir_force = min(max_dir_force, max_force(sc, J, angle))
    return max_dir_force


def max_force(sc: Scenario, J, angle):
    """Solves torque needed for each direction.
    increase the magnitue of the force until T1 or T2 > Tmax."""
    max_t = sc.Tmax * sc.ratio  # max torque Nm
    mag = 0
    while True:
        R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        f = mag * R @ np.array([[1], [0]])
        torque = np.transpose(J) @ f
        if abs(torque[0].item()) >= max_t or abs(torque[1].item()) >= max_t:
            return mag
        mag = mag + 0.01
