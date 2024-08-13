import warnings
from dataclasses import dataclass
from typing import Any

import gtsam
import numpy as np
import visgeom as vg
from shapely import geometry as sg, ops as so
import matplotlib.pyplot as plt
import matplotlib


def plot_result(poses, landmarks):
    matplotlib.use('qt5agg')

    plt.figure()
    ax = plt.axes()

    num_draws = 1000

    # Plot pose marginals.
    num_poses = len(poses)
    pose_cmap = plt.get_cmap('autumn')
    for dist, color in zip(poses, pose_cmap([number / num_poses for number in range(num_poses)])):
        mean_translation = dist.mean.translation()

        # Plot covariance from manifold.
        plot_se2_covariance_on_manifold_gtsam(ax, dist, fill_alpha=0.5, fill_color=color)

        # Plot random points from distribution.
        random_points = np.random.multivariate_normal(np.zeros(3), dist.covariance, num_draws).T
        random_translations = np.zeros([2, num_draws])
        for i in range(0, num_draws):
            random_translations[:, i] = dist.mean.compose(gtsam.Pose2.Expmap(random_points[:, i])).translation()

        ax.plot(random_translations[0, :], random_translations[1, :],
                marker='.', markeredgewidth=0, color=color, linestyle='', alpha=0.2)

        # Plot mean.
        ax.plot(mean_translation[0], mean_translation[1], marker='+', color='k', linestyle='')

    # Plot landmark marginals.
    num_landmarks = len(landmarks)
    landmark_cmap = plt.get_cmap('summer')
    for dist, color in zip(landmarks, landmark_cmap([number / num_landmarks for number in range(num_landmarks)])):
        plot_ellipse(ax, dist, fill_alpha=0.5, fill_color=color)

        random_points = np.random.multivariate_normal(dist.mean, dist.covariance, num_draws).T
        ax.plot(random_points[0, :], random_points[1, :],
                marker=".", markeredgewidth=0, color=color, linestyle='', alpha=0.2)

        ax.plot(dist.mean[0], dist.mean[1], marker='+', color='k', linestyle='')

    ax.set_aspect('equal', adjustable='box')

    plt.show()


def plot_se2_covariance_on_manifold_gtsam(ax, dist,
                                          n=50, chi2_val=11.345, right_perturbation=True,
                                          fill_alpha=0., fill_color='lightsteelblue'):
    u, s, _ = np.linalg.svd(dist.covariance)
    scale = np.sqrt(chi2_val * s)

    x, y, z = vg.utils.generate_ellipsoid(n, pose=(u, np.zeros([3, 1])), scale=scale)

    tangent_points = np.vstack((x.flatten(), y.flatten(), z.flatten()))

    num_samples = tangent_points.shape[1]
    transl_points = np.zeros([num_samples, 2])

    if right_perturbation:
        for i in range(num_samples):
            transl_points[i, :] = dist.mean.compose(gtsam.Pose2.Expmap(tangent_points[:, i])).translation()
    else:
        for i in range(num_samples):
            transl_points[i, :] = gtsam.Pose2.Expmap(tangent_points[:, i]).compose(dist.mean).translation()

    p_grid = np.reshape(transl_points, [(n + 1), (n + 1), 2])
    polygons = extract_polygon_slices(p_grid)
    union = so.unary_union(polygons)
    if not union.geom_type == 'Polygon':
        warnings.warn("Could not find a closed boundary", RuntimeWarning)
        return

    ax.fill(*union.exterior.xy, alpha=fill_alpha, facecolor=fill_color)
    ax.plot(*union.exterior.xy, color=fill_color)


def extract_polygon_slices(grid_2d):
    p_a = grid_2d[:-1, :-1]
    p_b = grid_2d[:-1, 1:]
    p_c = grid_2d[1:, 1:]
    p_d = grid_2d[1:, :-1]

    quads = np.concatenate((p_a, p_b, p_c, p_d), axis=2)

    m, n, _ = grid_2d.shape
    quads = quads.reshape(((m-1) * (n-1), 4, 2))

    return [sg.Polygon(t).buffer(0.0001, cap_style=2, join_style=2) for t in quads]


def plot_ellipse(ax, dist, n=50, chi2_val=9.21, fill_alpha=0., fill_color='lightsteelblue'):
    u, s, _ = np.linalg.svd(dist.covariance)
    scale = np.sqrt(chi2_val * s)

    theta = np.linspace(0, 2*np.pi, n+1)
    x = np.cos(theta)
    y = np.sin(theta)

    R = u
    t = np.reshape(dist.mean, [2, 1])
    circle_points = (R * scale) @ np.vstack((x.flatten(), y.flatten())) + t

    ax.fill(circle_points[0, :], circle_points[1, :], alpha=fill_alpha, facecolor=fill_color)
    ax.plot(circle_points[0, :], circle_points[1, :], color=fill_color)


@dataclass
class MultivariateNormalParameters:
    mean: Any
    covariance: np.ndarray