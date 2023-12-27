package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.path.Path100;
import org.team100.lib.spline.HolonomicSpline;
import org.team100.lib.spline.SplineGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryUtil100 {

    public static Path100 trajectoryFromWaypointsAndHeadings(
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            double maxDx,
            double maxDy,
            double maxDTheta) {
        List<HolonomicSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new HolonomicSpline(
                    waypoints.get(i - 1), waypoints.get(i),
                    headings.get(i - 1), headings.get(i)));
        }
        HolonomicSpline.optimizeSpline(splines);
        return new Path100(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy, maxDTheta));
    }

    private TrajectoryUtil100() {
    }
}
