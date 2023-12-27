package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);

    private final SwerveKinodynamics m_limits;

    public TrajectoryPlanner(SwerveKinodynamics limits) {
        m_limits = limits;
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double max_vel,
            double max_accel) {
        return generateTrajectory(
                reversed, waypoints,
                headings,
                constraints,
                0.0,
                0.0,
                max_vel,
                max_accel);
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel,
            double max_accel) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        List<Rotation2d> headings_maybe_flipped = headings;
        final Pose2d flip = GeometryUtil.fromRotation(new Rotation2d(-1, 0));
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            headings_maybe_flipped = new ArrayList<>(headings.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(GeometryUtil.transformBy(waypoints.get(i), flip));
                headings_maybe_flipped.add(headings.get(i).rotateBy(flip.getRotation()));
            }
        }

        // Create a trajectory from splines.
        Path100 trajectory = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(
                waypoints_maybe_flipped, headings_maybe_flipped, 
                kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithMotion> flipped_points = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped_points.add(
                        new Pose2dWithMotion(GeometryUtil.transformBy(trajectory.getPoint(i).state().getPose(), flip),
                                -trajectory
                                        .getPoint(i).state().getCurvature(),
                                trajectory.getPoint(i).state().getDCurvatureDs()));
            }
            trajectory = new Path100(flipped_points);
        }

        final SwerveDriveDynamicsConstraint drive_constraints = new SwerveDriveDynamicsConstraint(m_limits);
        final YawRateConstraint yaw_constraint = new YawRateConstraint(m_limits);

        final CentripetalAccelerationConstraint centripetal_accel_constraint = new CentripetalAccelerationConstraint(
                m_limits);

        List<TimingConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        all_constraints.add(yaw_constraint);
        all_constraints.add(centripetal_accel_constraint);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        PathDistanceSampler distance_view = new PathDistanceSampler(trajectory);
        return TimingUtil.timeParameterizeTrajectory(
                reversed,
                distance_view,
                kMaxDx,
                all_constraints,
                start_vel,
                end_vel,
                max_vel,
                max_accel);
    }
}
