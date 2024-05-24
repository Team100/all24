package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * joel 20240311: this class no longer applies default constraints (drive, yaw,
 * centripetal) so if you want those, supply them.
 */
public class TrajectoryPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);

    public static Trajectory100 generateTrajectory(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_accel) {
        // Create a path from splines.
        Path100 path = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                waypoints, headings, kMaxDx, kMaxDy, kMaxDTheta);
        // Generate the timed trajectory.
        PathDistanceSampler distance_view = new PathDistanceSampler(path);
        return TimingUtil.timeParameterizeTrajectory(
                distance_view,
                kMaxDx,
                constraints,
                start_vel,
                end_vel,
                max_vel,
                max_accel);
    }

    private TrajectoryPlanner() {
        //
    }
}
