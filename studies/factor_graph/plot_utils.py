# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


import math
import gtsam  # type:ignore
import matplotlib

matplotlib.use("Qt5Agg", force=True)
import matplotlib.pyplot as plt  # type:ignore
from matplotlib.markers import MarkerStyle
from matplotlib import collections
import numpy as np
from gtsam.symbol_shorthand import X  # type:ignore
from landmark import Landmark


NUM_DRAWS = 1000
TAG_SCALE = 0.1
POSE = 0.1

landmark_mark = np.array(
    [
        [-TAG_SCALE, -TAG_SCALE],
        [TAG_SCALE, -TAG_SCALE],
        [TAG_SCALE, TAG_SCALE],
        [-TAG_SCALE, TAG_SCALE],
    ]
)
pose_mark = np.array(
    [
        [-POSE, POSE],
        [-POSE / 2, 0],
        [-POSE, -POSE],
        [POSE, 0],
    ]
)


def rot(theta):
    return np.array(
        [
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)],
        ]
    )


def make_mark(mark, rot, c):
    return mark.dot(rot) + c


class Plot:
    @staticmethod
    def fig(name):
        return plt.figure(name, figsize=(6, 6))

    @staticmethod
    def subplots(x, y, sx, sy):
        return plt.subplots(x, y, figsize=(sx, sy))

    def __init__(self, isam, name, fig, ax) -> None:
        self.isam = isam
        self.rng = np.random.default_rng(0)
        self.fig = fig
        self.ax = ax
        self.ax.set_title(name)
        self.ax.set_xlim(-1, 6)
        self.ax.set_ylim(-1, 6)
        self.ax.set_aspect("equal", adjustable="box")
        plt.tight_layout()
        self.pose_mean_poly = collections.PolyCollection(
            [],
            facecolors="none",
            edgecolors="black",
        )
        self.ax.add_collection(self.pose_mean_poly)
        self.pose_point_scatter = self.ax.scatter(
            [],
            [],
            marker=MarkerStyle("x"),
            s=5,
            alpha=0.1,
            color="orange",
            linewidths=1,
            zorder=1,
        )
        self.landmark_mean_poly = collections.PolyCollection(
            [],
            facecolors="none",
            edgecolors="black",
        )
        self.ax.add_collection(self.landmark_mean_poly)
        self.landmark_point_scatter = self.ax.scatter(
            [],
            [],
            marker=MarkerStyle("x"),
            s=5,
            alpha=0.1,
            color="royalblue",
            linewidths=1,
            zorder=1,
        )
        self.fig.canvas.draw()
        plt.show(block=False)
        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def plot_variables(
        self, result: gtsam.Values, poses: list[X], landmarks: list[Landmark]
    ) -> None:
        self.ax.draw_artist(self.ax.patch)

        if len(poses) > 0:
            pose_mean_poses = np.array([result.atPose2(var) for var in poses])
            pose_mean_verts = [
                make_mark(pose_mark, rot(c.theta()), c.translation())
                for c in pose_mean_poses
            ]
            self.pose_mean_poly.set_verts(pose_mean_verts)
            
            pose_point_translations = np.vstack(
                [self.pose_point(result, var) for var in poses]
            )
            self.pose_point_scatter.set_offsets(pose_point_translations)

            # poly is drawn on top of scatter
            self.ax.draw_artist(self.pose_point_scatter)
            self.ax.draw_artist(self.pose_mean_poly)

        if len(landmarks) > 0:
            landmark_mean_translations = np.array(
                [result.atPoint2(var.symbol) for var in landmarks]
            )
            landmark_mean_verts = [
                make_mark(landmark_mark, rot(math.pi / 4), c)
                for c in landmark_mean_translations
            ]
            self.landmark_mean_poly.set_verts(landmark_mean_verts)

            landmark_point_translations = np.vstack(
                [self.landmark_point(result, var.symbol) for var in landmarks]
            )
            self.landmark_point_scatter.set_offsets(landmark_point_translations)

            self.ax.draw_artist(self.landmark_point_scatter)
            self.ax.draw_artist(self.landmark_mean_poly)

        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def landmark_point(self, result, var):
        mean = result.atPoint2(var)  # 1x2
        covariance = self.isam.getISAM2().marginalCovariance(var)  # 2x2
        return self.rng.multivariate_normal(mean, covariance, NUM_DRAWS)

    def pose_point(self, result: gtsam.Values, var) -> np.ndarray:
        mean: gtsam.Pose2 = result.atPose2(var)  # 1x3
        # this covariance is in the *tangent space* at the *mean*
        # i.e. the "twist", so that's how we apply it
        # the getISAM2 thing is because the python wrapper doesn't do it
        # for the fixed lag smoother
        covariance: np.ndarray = self.isam.getISAM2().marginalCovariance(var)  # 3x3

        random_points: np.ndarray = self.rng.multivariate_normal(
            np.zeros(3), covariance, NUM_DRAWS
        )
        random_translations: np.ndarray = np.zeros([NUM_DRAWS, 2])
        for i in range(0, NUM_DRAWS):
            random_translations[i, :] = mean.compose(
                gtsam.Pose2.Expmap(random_points[i, :])
            ).translation()
        return random_translations
