package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.SwerveDriveDynamicsConstraint;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class TrajectoryPlanner {
    private static final double kMaxDx = 0.0127; // m
    private static final double kMaxDy = 0.0127; // m
    private static final double kMaxDTheta = Math.toRadians(1.0);

    private final SwerveDriveKinematics m_kinematics;
    private final SwerveKinematicLimits m_limits;

    public TrajectoryPlanner(SwerveDriveKinematics kinematics, SwerveKinematicLimits limits) {
        m_kinematics = kinematics;
        m_limits = limits;
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double max_vel, // m/s
            double max_accel, // m/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, headings, constraints, 0.0, 0.0, max_vel, max_accel,
                max_voltage);
    }

    public Trajectory100 generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint> constraints,
            double start_vel,
            double end_vel,
            double max_vel, // m/s
            double max_accel, // m/s^2
            double max_voltage) {
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
                waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

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

        // Create the constraint that the robot must be able to traverse the trajectory
        // without ever applying more
        // than the specified voltage.

        // TODO: move these constant constraints to the constructor
        final SwerveDriveDynamicsConstraint drive_constraints = new SwerveDriveDynamicsConstraint(m_kinematics,
                m_limits);
        final double kMaxYawRateRadS = 3.0;
        final YawRateConstraint yaw_constraint = new YawRateConstraint(kMaxYawRateRadS);
        final double kMaxCentripetalAccel = 10.0;// 1.524; // m/s^2
        final CentripetalAccelerationConstraint centripetal_accel_constraint = new CentripetalAccelerationConstraint(
                kMaxCentripetalAccel);

        List<TimingConstraint> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        all_constraints.add(yaw_constraint);
        all_constraints.add(centripetal_accel_constraint);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        PathDistanceSampler distance_view = new PathDistanceSampler(trajectory);
        return TimingUtil.timeParameterizeTrajectory(reversed,
                distance_view, kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
    }
}
