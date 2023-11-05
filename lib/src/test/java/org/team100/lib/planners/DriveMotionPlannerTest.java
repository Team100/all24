package org.team100.lib.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.path.Path;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.path.PathIndexSampler;
import org.team100.lib.paths.TrajectoryGenerator;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.trajectory.Trajectory;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

class DriveMotionPlannerTest {
    public static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;

    public static final double kDriveTrackwidthMeters = 0.52705; // DONE Measure and set trackwidth
    public static final double kDriveWheelbaseMeters = 0.52705; // DONE Measure and set wheelbase

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(kDriveTrackwidthMeters / 2.0, -kDriveWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-kDriveTrackwidthMeters / 2.0, kDriveWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-kDriveTrackwidthMeters / 2.0, -kDriveWheelbaseMeters / 2.0));

    public static final SwerveKinematicLimits kSmoothKinematicLimits = new SwerveKinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond * .9;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kMaxAccelerationMetersPerSecondSquared;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    @Test
    void testTrajectory() {
        List<Pose2d> waypoints = new ArrayList<>();
        List<Rotation2d> headings = new ArrayList<>();
        waypoints.add(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(100, 4, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(180));
        waypoints.add(new Pose2d(196, 13, Rotation2d.fromDegrees(0)));
        headings.add(Rotation2d.fromDegrees(0));

        double start_vel = 0.0;
        double end_vel = 0.0;
        double max_vel = 100;
        double max_accel = 100;

        Path traj = new Path();
        Assertions.assertTrue(traj.isEmpty());
        Assertions.assertEquals(0.0, new PathIndexSampler(traj).first_interpolant(), 0.2);
        Assertions.assertEquals(0.0, new PathIndexSampler(traj).last_interpolant(), 0.2);
        Assertions.assertEquals(0, traj.length());

        // Set states at construction time.
        traj = TrajectoryUtil.trajectoryFromWaypointsAndHeadings(waypoints, headings, 2, 0.25, 0.1);
        Assertions.assertFalse(traj.isEmpty());
        Assertions.assertEquals(0.0, new PathIndexSampler(traj).first_interpolant(), 0.2);
        // Assertions.assertEquals(3.0, traj.getIndexView().last_interpolant(), 0.2);
        // Assertions.assertEquals(3, traj.length());

        // for(int i = 0; i < traj.length(); i++) {
        // System.out.println(traj.getPoint(i).toString());
        // System.out.println(traj.getPoint(i).state().toString());
        // System.out.println(traj.getPoint(i).index());
        // }

        Trajectory timed_trajectory = TimingUtil.timeParameterizeTrajectory(false, new PathDistanceSampler(traj), 2,
                Arrays.asList(), start_vel, end_vel, max_vel, max_accel);

        // System.out.println("\n\n\n\n\n\n\n");

        // for(int i = 0; i < timed_trajectory.length(); i++) {
        // System.out.println(timed_trajectory.getPoint(i).toString());
        // System.out.println(timed_trajectory.getPoint(i).state().state().toString());
        // System.out.println(timed_trajectory.getPoint(i).index());
        // }

        DriveMotionPlanner planner = new DriveMotionPlanner(kKinematics, kSmoothKinematicLimits);
        TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(timed_trajectory));
        planner.setTrajectory(traj_iterator);

        Pose2d pose = timed_trajectory.getPoint(0).state().state().getPose();
        Twist2d velocity = new Twist2d();

        double time = 0.0;
        double mDt = 0.005;
        while (!planner.isDone()) {
            ChassisSpeeds speeds = planner.update(time, pose, velocity);
            Twist2d twist = new Twist2d(speeds.vxMetersPerSecond * mDt, speeds.vyMetersPerSecond * mDt,
                    speeds.omegaRadiansPerSecond * mDt);
            velocity = GeometryUtil.toTwist2d(speeds);
            pose = GeometryUtil.transformBy(pose, new Pose2d().exp(twist));
            // System.out.println("\n\n\n\n-----t="+time);
            // System.out.println(speeds);
            // System.out.println(pose);
            // System.out.println("Pathsetpoint:" + planner.getSetpoint());
            time += mDt;
        }
        Assertions.assertEquals(196, pose.getTranslation().getX(), 0.2);
        Assertions.assertEquals(13, pose.getTranslation().getY(), 0.1);
        Assertions.assertEquals(0, pose.getRotation().getDegrees(), 0.4);
    }

    @Test
    void testAllTrajectories() {
        SwerveDriveKinematics kinematics = kKinematics;
        SwerveKinematicLimits limits = kSmoothKinematicLimits;
        DriveMotionPlanner planner = new DriveMotionPlanner(kinematics, limits);
        TrajectoryGenerator generator = new TrajectoryGenerator(planner);
        generator.generateTrajectories();
        var trajectories = generator.getTrajectorySet().getAllTrajectories();

        // AsymSwerveSetpointGenerator setpoint_generator = new AsymSwerveSetpointGenerator(kinematics);

        for (var traj : trajectories) {
            // System.out.println("\n" + traj.toString());
            planner.reset();
            TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                    new TrajectoryTimeSampler(traj));
            planner.setTrajectory(traj_iterator);
            final Pose2d kInjectedError = new Pose2d(0.3, -0.1, Rotation2d.fromDegrees(9.0));
            final Twist2d kInjectedVelocityError = new Twist2d(0.1, 0.3, 0.0);
            final double kInjectionTime = 20.0;
            Pose2d pose = traj.getPoint(0).state().state().getPose();
            Twist2d velocity = new Twist2d();
            SwerveSetpoint setpoint = null;
            double time = 0.0;
            double mDt = 0.005;
            boolean error_injected = false;
            while (!planner.isDone()) {
                // System.out.println("\n\n-----t=" + time);
                if (!error_injected && time >= kInjectionTime) {
                    pose = GeometryUtil.transformBy(pose, kInjectedError);
                    velocity = new Twist2d(velocity.dx + kInjectedVelocityError.dx,
                            velocity.dy + kInjectedVelocityError.dy, velocity.dtheta + kInjectedVelocityError.dtheta);
                    error_injected = true;
                }
                ChassisSpeeds speeds = planner.update(time, pose, velocity);
                if (true) {// setpoint == null) {
                    // Initialilze from first chassis speeds.
                    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
                    setpoint = new SwerveSetpoint(speeds, states);
                }
                // setpoint = setpoint_generator.generateSetpoint(limits, setpoint, speeds,
                // mDt);
                // Don't use a twist here (assume Drive compensates for that)
                Pose2d delta = new Pose2d(
                    new Translation2d(
                        setpoint.getChassisSpeeds().vxMetersPerSecond * mDt,
                        setpoint.getChassisSpeeds().vyMetersPerSecond * mDt),
                        Rotation2d.fromRadians(setpoint.getChassisSpeeds().omegaRadiansPerSecond * mDt));
                pose = GeometryUtil.transformBy(pose, delta);
                velocity = GeometryUtil.toTwist2d(setpoint.getChassisSpeeds());

                // System.out.println("\n\n-----t="+time);
                // System.out.println(speeds);
                // System.out.println(pose);
                // System.out.println("Setpoint:" + planner.getSetpoint());
                // Inches and degrees
                Pose2d error = GeometryUtil.transformBy(GeometryUtil.inverse(pose), planner.getSetpoint().state().getPose());
                // System.out.println("Setpoint: " + planner.getSetpoint());
                // System.out.println("Error: " + error);
                Assertions.assertEquals(0.0, error.getTranslation().getX(), 0.0508);
                Assertions.assertEquals(0.0, error.getTranslation().getY(), 0.0508);
                Assertions.assertEquals(0.0, error.getRotation().getDegrees(), 5.0);

                time += mDt;
            }
        }
    }
}