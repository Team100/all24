# pylint: disable=wrong-import-position,invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


import math
import gtsam  # type:ignore
import matplotlib  # type:ignore

matplotlib.use("Qt5Agg", force=True)
import matplotlib.pyplot as plt  # type:ignore
from matplotlib.markers import MarkerStyle  # type:ignore
from matplotlib import collections
import numpy as np
from gtsam.symbol_shorthand import X  # type:ignore
from landmark import Landmark


NUM_DRAWS = 50
TAG_SCALE = 0.1
POSE = 0.1
RNG = np.random.default_rng(0)


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


def make_mark(mark, rot_matrix, c):
    return mark.dot(rot_matrix) + c


class Plot:
    @staticmethod
    def subplots(x, y, sx, sy):
        return plt.subplots(x, y, figsize=(sx, sy))

    def __init__(self, isam, name, fig, ax) -> None:
        self.isam = isam
        self.fig = fig
        self.ax = ax
        self.ax.set_title(name)
        self.ax.set_xlim(-1, 6)
        self.ax.set_ylim(-1, 6)
        self.ax.set_aspect("equal", adjustable="box")
        plt.tight_layout()

        # the order the collections are added to the axis
        # is the order they are drawn below, so put the
        # mean after the distribution.
        self.pose_point_scatter = self.ax.scatter(
            [],
            [],
            marker=MarkerStyle("x"),
            s=5,
            alpha=0.5,
            color="orange",
        )
        self.pose_mean_poly = collections.PolyCollection(
            [],
            facecolors="none",
            edgecolors="black",
        )
        self.ax.add_collection(self.pose_mean_poly)

        self.landmark_point_scatter = self.ax.scatter(
            [],
            [],
            marker=MarkerStyle("x"),
            s=5,
            alpha=0.5,
            color="royalblue",
        )
        self.landmark_mean_poly = collections.PolyCollection(
            [],
            facecolors="none",
            edgecolors="black",
        )
        self.ax.add_collection(self.landmark_mean_poly)

        self.sights = collections.PathCollection(
            [],
            edgecolors="green",
        )
        self.ax.add_collection(self.sights)

        self.fig.canvas.draw()
        plt.show(block=False)
        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    @staticmethod
    def make_lines(pose_mean_poses):
        c: gtsam.Pose2
        for c in pose_mean_poses:
            for l in [[0.5, 0.5], [0.5, 4.5]]:
                # TODO: make this somehow use the factors that exist
                l_angle = c.bearing(l)
                l_range = c.range(l)
                if abs(l_angle.theta()) < 0.5 and l_range < 4:
                    yield matplotlib.path.Path([l, c.translation()])
        # paths = [
        #     matplotlib.path.Path([[0.5, 0.5], c.translation()]) for c in pose_mean_poses
        # ]
        # return paths

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

            try:
                pose_point_translations = np.vstack(
                    [self.pose_point(result, var) for var in poses]
                )
                self.pose_point_scatter.set_offsets(pose_point_translations)
            except AttributeError:
                pass # no marginal covariance

            # also make the lines to the landmarks
            # these won't be gtsam variables soon
            # lines = [[[0.5, 0.5], c.translation().tolist()] for c in pose_mean_poses]
            # lines = [[(0,0),(1,1)]]
            # paths = [matplotlib.path.Path([[1,2],[3,4]])]
            paths = list(self.make_lines(pose_mean_poses))
            self.sights.set_paths(paths)

        if len(landmarks) > 0:
            landmark_mean_translations = np.array(
                [result.atPoint2(var.symbol) for var in landmarks]
            )
            landmark_mean_verts = [
                make_mark(landmark_mark, rot(math.pi / 4), c)
                for c in landmark_mean_translations
            ]
            self.landmark_mean_poly.set_verts(landmark_mean_verts)

            try:
                landmark_point_translations = np.vstack(
                    [self.landmark_point(result, var.symbol) for var in landmarks]
                )
                self.landmark_point_scatter.set_offsets(landmark_point_translations)
            except AttributeError:
                pass # batch smoother has no marginal covariance method.

        self.ax.redraw_in_frame()
        self.fig.canvas.update()
        self.fig.canvas.flush_events()

    def landmark_point(self, result, var):
        mean = result.atPoint2(var)  # 1x2
        covariance = self.isam.getISAM2().marginalCovariance(var)  # 2x2
        return RNG.multivariate_normal(mean, covariance, NUM_DRAWS)

    def pose_point(self, result: gtsam.Values, var) -> np.ndarray:
        mean: gtsam.Pose2 = result.atPose2(var)  # 1x3
        # this covariance is in the *tangent space* at the *mean*
        # i.e. the "twist", so that's how we apply it
        # the getISAM2 thing is because the python wrapper doesn't do it
        # for the fixed lag smoother
        covariance: np.ndarray = self.isam.getISAM2().marginalCovariance(var)  # 3x3

        # print(covariance)
        random_points: np.ndarray = RNG.multivariate_normal(
            np.zeros(3), covariance, NUM_DRAWS
        )
        random_translations: np.ndarray = np.zeros([NUM_DRAWS, 2])
        for i in range(0, NUM_DRAWS):
            random_translations[i, :] = mean.compose(
                gtsam.Pose2.Expmap(random_points[i, :])
            ).translation()
        return random_translations
