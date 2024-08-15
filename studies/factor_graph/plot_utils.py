# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


import gtsam  # type:ignore
import matplotlib.pyplot as plt  # type:ignore
import numpy as np
from gtsam.symbol_shorthand import X, L  # type:ignore

NUM_DRAWS = 1000


class Plot:

    def __init__(self, isam) -> None:
        self.isam = isam
        self.rng = np.random.default_rng(0)
        self.fig = plt.figure("click to close this window", figsize=(10, 10))
        self.ax = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
        self.ax.set_aspect("equal", adjustable="box")

        self.pose_mean_scatter = self.ax.scatter(
            [], [], marker="+", s=200, color="black", linewidths=0.5, zorder=20
        )
        self.pose_point_scatter = self.ax.scatter(
            [],
            [],
            marker="x",
            s=5,
            alpha=0.1,
            color="orange",
            linewidths=1,
            zorder=1,
        )
        self.landmark_mean_scatter = self.ax.scatter(
            [], [], marker="+", s=200, color="black", linewidths=0.5, zorder=20
        )
        self.landmark_point_scatter = self.ax.scatter(
            [],
            [],
            marker="o",
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

    def plot_variables(self, result, poses: list[X], landmarks: list[L]) -> None:
        self.ax.draw_artist(self.ax.patch)

        if len(poses) > 0:
            pose_mean_translations = np.array(
                [result.atPose2(var).translation() for var in poses]
            )

            pose_point_translations = np.vstack(
                [self.pose_point(result, var) for var in poses]
            )

            self.pose_mean_scatter.set_offsets(pose_mean_translations)
            self.pose_point_scatter.set_offsets(pose_point_translations)

            self.ax.draw_artist(self.pose_point_scatter)
            self.ax.draw_artist(self.pose_mean_scatter)

        if len(landmarks) > 0:
            landmark_mean_translations = np.array(
                [result.atPoint2(var) for var in landmarks]
            )

            landmark_point_translations = np.vstack(
                [self.landmark_point(result, var) for var in landmarks]
            )

            self.landmark_mean_scatter.set_offsets(landmark_mean_translations)
            self.landmark_point_scatter.set_offsets(landmark_point_translations)

            self.ax.draw_artist(self.landmark_point_scatter)
            self.ax.draw_artist(self.landmark_mean_scatter)

        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def landmark_point(self, result, var):
        mean = result.atPoint2(var)  # 1x2
        covariance = self.isam.getISAM2().marginalCovariance(var)  # 2x2
        return self.rng.multivariate_normal(mean, covariance, NUM_DRAWS)

    def pose_point(self, result, var):
        mean = result.atPose2(var)  # 1x3
        # this covariance is in the *tangent space* at the *mean*
        # i.e. the "twist", so that's how we apply it
        # the getISAM2 thing is because the python wrapper doesn't do it
        # for the fixed lag smoother
        covariance = self.isam.getISAM2().marginalCovariance(var)  # 3x3

        random_points = self.rng.multivariate_normal(np.zeros(3), covariance, NUM_DRAWS)
        random_translations = np.zeros([NUM_DRAWS, 2])
        for i in range(0, NUM_DRAWS):
            random_translations[i, :] = mean.compose(
                gtsam.Pose2.Expmap(random_points[i, :])
            ).translation()
        return random_translations
