package org.team100.lib.trajectory;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.planners.DriveMotionPlanner;
import org.team100.lib.swerve.SwerveKinematicLimits;
import org.team100.lib.timing.CentripetalAccelerationConstraint;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

class FancyTrajectoryTest {

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
    void testLikeAuton() {
        final double kMaxVel = 1.0;
        final double kMaxAccel = 1.0;
        // this doesn't actually do anything.
        final double kMaxVoltage = 9.0;

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or less.
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner(kKinematics, kSmoothKinematicLimits);
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner.generateTrajectory(
                false,
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVel,
                kMaxAccel,
                kMaxVoltage);
        // System.out.println(trajectory);
        // System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // why is this so large?
        assertEquals(1300, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            // System.out.println("============initialize============");
            ChassisSpeeds output = mMotionPlanner.update(0,
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                    new Twist2d());
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0, output.omegaRadiansPerSecond, 0.001);
        }

        {
            // System.out.println("============4 sec============");
            ChassisSpeeds output = mMotionPlanner.update(4.0,
                    new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69)),
                    new Twist2d());
            // remember, facing +90, moving -90, so this should be like -1
            assertEquals(-1, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.1, output.vyMetersPerSecond, 0.05);
            // turning slowly to the left
            assertEquals(0.1, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.05);
            assertEquals(0, translational_error.getY(), 0.05);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.05);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(0.25, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(1, path_setpoint.velocityM_S(), 0.01);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
        {
            // System.out.println("============8 sec============");
            ChassisSpeeds output = mMotionPlanner.update(8.0,
                    new Pose2d(new Translation2d(1.85, -7.11), Rotation2d.fromRadians(2.22)),
                    new Twist2d());
            assertEquals(-0.96, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.05, output.vyMetersPerSecond, 0.05);
            assertEquals(0.18, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.01);
            assertEquals(0, translational_error.getY(), 0.01);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.01);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(1.85, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-7.11, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.22, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(1, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
    }


    @Test
    void testPursuit() {
        final double kMaxVel = 1.0;
        final double kMaxAccel = 1.0;
        // this doesn't actually do anything.
        final double kMaxVoltage = 9.0;

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or less.
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner(kKinematics, kSmoothKinematicLimits);
        mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.PURE_PURSUIT);
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner.generateTrajectory(
                false,
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVel,
                kMaxAccel,
                kMaxVoltage);
        // System.out.println(trajectory);
        // System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // why is this so large?
        assertEquals(1300, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            // System.out.println("============initialize============");
            ChassisSpeeds output = mMotionPlanner.update(0,
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                    new Twist2d());
                    // this is default cook.
                    // TODO: remove that idea.
            assertEquals(-2.48, output.vxMetersPerSecond, 0.05);
            assertEquals(0, output.vyMetersPerSecond, 0.05);
            // omega is NaN, i think pursuit ignores omega, it uses feedforward only.
            // assertEquals(0, output.omegaRadiansPerSecond, 0.001);
        }

        {
            // System.out.println("============4 sec============");
            ChassisSpeeds output = mMotionPlanner.update(4.0,
                    new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69)),
                    new Twist2d());
            // remember, facing +90, moving -90, so this should be like -1
            // but actually it's default cook.
            assertEquals(-2.48, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.15, output.vyMetersPerSecond, 0.05);
            // turning slowly to the left
            // i think pure pursuit might ignore omega
            assertEquals(0, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.05);
            assertEquals(0, translational_error.getY(), 0.05);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.05);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(0.25, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.05);
            assertEquals(1, path_setpoint.velocityM_S(), 0.01);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
        {
            // System.out.println("============8 sec============");
            ChassisSpeeds output = mMotionPlanner.update(8.0,
                    new Pose2d(new Translation2d(1.85, -7.11), Rotation2d.fromRadians(2.22)),
                    new Twist2d());
                    // this is default cook again
            assertEquals(-2.5, output.vxMetersPerSecond, 0.05);
            // this is more Y than PID because it looks ahead
            assertEquals(-0.15, output.vyMetersPerSecond, 0.05);
            assertEquals(0, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.05);
            assertEquals(0, translational_error.getY(), 0.01);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.01);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(1.85, path_setpoint.state().getPose().getX(), 0.05);
            assertEquals(-7.11, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.22, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.05);
            assertEquals(1, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
    }

    @Test
    void testFeedforwardOnly() {
        final double kMaxVel = 1.0;
        final double kMaxAccel = 1.0;
        // this doesn't actually do anything.
        final double kMaxVoltage = 9.0;

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or less.
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner(kKinematics, kSmoothKinematicLimits);
        mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner.generateTrajectory(
                false,
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVel,
                kMaxAccel,
                kMaxVoltage);
        // System.out.println(trajectory);
        // System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // why is this so large?
        assertEquals(1300, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            // System.out.println("============initialize============");
            ChassisSpeeds output = mMotionPlanner.update(0,
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                    new Twist2d());
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0, output.omegaRadiansPerSecond, 0.001);
        }

        {
            // System.out.println("============4 sec============");
            ChassisSpeeds output = mMotionPlanner.update(4.0,
                    new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69)),
                    new Twist2d());
            // remember, facing +90, moving -90, so this should be like -1
            assertEquals(-1, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.1, output.vyMetersPerSecond, 0.05);
            // turning slowly to the left
            assertEquals(0.1, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.05);
            assertEquals(0, translational_error.getY(), 0.05);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.05);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(0.25, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(1, path_setpoint.velocityM_S(), 0.01);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
        {
            // System.out.println("============8 sec============");
            ChassisSpeeds output = mMotionPlanner.update(8.0,
                    new Pose2d(new Translation2d(1.85, -7.11), Rotation2d.fromRadians(2.22)),
                    new Twist2d());
            assertEquals(-0.96, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.05, output.vyMetersPerSecond, 0.05);
            assertEquals(0.18, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.01);
            assertEquals(0, translational_error.getY(), 0.01);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.01);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(1.85, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-7.11, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.22, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(1, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
    }
    @Test
    void testRamsete() {
        // i don't understand how ramsete is relevant here
        final double kMaxVel = 1.0;
        final double kMaxAccel = 1.0;
        // this doesn't actually do anything.
        final double kMaxVoltage = 9.0;

        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or less.
        // these don't actually do anything.
        List<TimingConstraint> constraints = List.of(
                new CentripetalAccelerationConstraint(60));

        // note there are static constraints in here.
        DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner(kKinematics, kSmoothKinematicLimits);
        mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.RAMSETE);
        double start_vel = 0;
        double end_vel = 0;
        // there's a bug in here; it doesn't use the constraints, nor the voltage.
        Trajectory trajectory = mMotionPlanner.generateTrajectory(
                false,
                waypoints,
                headings,
                constraints,
                start_vel,
                end_vel,
                kMaxVel,
                kMaxAccel,
                kMaxVoltage);
        // System.out.println(trajectory);
        // System.out.println("TRAJECTORY LENGTH: " + trajectory.length());
        // why is this so large?
        assertEquals(1300, trajectory.length());

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);

        mMotionPlanner.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            // System.out.println("============initialize============");
            ChassisSpeeds output = mMotionPlanner.update(0,
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                    new Twist2d());
            assertEquals(0, output.vxMetersPerSecond, 0.001);
            assertEquals(0, output.vyMetersPerSecond, 0.001);
            assertEquals(0, output.omegaRadiansPerSecond, 0.001);
        }

        {
            // System.out.println("============4 sec============");
            ChassisSpeeds output = mMotionPlanner.update(4.0,
                    new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69)),
                    new Twist2d());
            // remember, facing +90, moving -90, so this should be like -1
            assertEquals(-1, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.1, output.vyMetersPerSecond, 0.05);
            // turning slowly to the left
            assertEquals(0.1, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.05);
            assertEquals(0, translational_error.getY(), 0.05);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.05);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(0.25, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(1, path_setpoint.velocityM_S(), 0.01);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
        {
            // System.out.println("============8 sec============");
            ChassisSpeeds output = mMotionPlanner.update(8.0,
                    new Pose2d(new Translation2d(1.85, -7.11), Rotation2d.fromRadians(2.22)),
                    new Twist2d());
            assertEquals(-0.96, output.vxMetersPerSecond, 0.05);
            assertEquals(-0.05, output.vyMetersPerSecond, 0.05);
            assertEquals(0.18, output.omegaRadiansPerSecond, 0.05);
            Translation2d translational_error = mMotionPlanner.getTranslationalError();
            assertEquals(0, translational_error.getX(), 0.01);
            assertEquals(0, translational_error.getY(), 0.01);
            Rotation2d heading_error = mMotionPlanner.getHeadingError();
            assertEquals(0, heading_error.getRadians(), 0.01);
            TimedPose path_setpoint = mMotionPlanner.getSetpoint();
            assertEquals(1.85, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-7.11, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.22, path_setpoint.state().getPose().getRotation().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(1, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);
            // Rotation2d heading_setpoint = mMotionPlanner.getHeadingSetpoint();
            // assertEquals(0, heading_setpoint.getRadians(), 0.001);
        }
    }
}
