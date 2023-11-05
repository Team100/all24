package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path;
import org.team100.lib.spline.PoseSpline;
import org.team100.lib.spline.QuinticHermitePoseSplineHolonomic;
import org.team100.lib.spline.QuinticHermitePoseSplineNonholonomic;
import org.team100.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryUtil {

    public static Path trajectoryFromWaypointsAndHeadings(final List<Pose2d> waypoints,
            final List<Rotation2d> headings, double maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermitePoseSplineNonholonomic> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermitePoseSplineHolonomic(waypoints.get(i - 1), waypoints.get(i),
                    headings.get(i - 1), headings.get(i)));
        }
        QuinticHermitePoseSplineHolonomic.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static Path trajectoryFromSplines(final List<? extends PoseSpline> splines, double maxDx, double maxDy,
            double maxDTheta) {
        List<Pose2dWithMotion> points = SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta);
        return new Path(points);
    }

    private TrajectoryUtil() {
    }
}
