package org.team100.lib.trajectory;

import java.util.List;

import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.util.Util;

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

    /**
     * If you want a max velocity or max accel constraint, use ConstantConstraint.
     */
    public static Trajectory100 generateTrajectory(
            List<Pose2d> waypoints,
            List<Rotation2d> headings,
            List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_accel) {
        try {
            // Create a path from splines.
            Path100 path = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                    waypoints, headings, kMaxDx, kMaxDy, kMaxDTheta);
            // Generate the timed trajectory.
            var view = new PathDistanceSampler(path);
            TimingUtil u = new TimingUtil(constraints, max_vel, max_accel);
            return u.timeParameterizeTrajectory(
                    view,
                    kMaxDx,
                    start_vel,
                    end_vel);
        } catch (IllegalArgumentException e) {
            // catches various kinds of malformed input, returns a no-op.
            // this should never actually happen.
            Util.warn("Bad trajectory input!!");
            e.printStackTrace();
            return new Trajectory100();
        }
    }

    private TrajectoryPlanner() {
        //
    }
}
