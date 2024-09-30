package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.path.Path100;
import org.team100.lib.path.PathDistanceSampler;
import org.team100.lib.swerve.SwerveSetpoint;
import org.team100.lib.timing.TimingUtil;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryGenerator100;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;
import org.team100.lib.trajectory.TrajectoryUtil100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DriveMotionPlannerTest {
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();
    private static final SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.get();

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

        Path100 traj = new Path100();
        assertTrue(traj.isEmpty());
        assertEquals(0, traj.length());

        // Set states at construction time.
        traj = TrajectoryUtil100.trajectoryFromWaypointsAndHeadings(waypoints, headings, 2, 0.25, 0.1);
        assertFalse(traj.isEmpty());

        var view = new PathDistanceSampler(traj);
        var stepSize = 2;
        TimingUtil u = new TimingUtil(Arrays.asList(), max_vel, max_accel);
        Trajectory100 timed_trajectory = u.timeParameterizeTrajectory(
                view,
                stepSize,
                start_vel,
                end_vel);

        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(logger);
        DriveTrajectoryFollower controller = new DrivePIDFFollower(PIDFlog, util, false, 2.4, 2.4);
        TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                new TrajectoryTimeSampler(timed_trajectory));
        controller.setTrajectory(traj_iterator);

        Pose2d pose = timed_trajectory.getPoint(0).state().state().getPose();
        ChassisSpeeds velocity = new ChassisSpeeds();

        double time = 0.0;
        double mDt = 0.005;
        while (!controller.isDone()) {
            ChassisSpeeds speeds = controller.update(time, pose, velocity);
            Twist2d twist = new Twist2d(
                    speeds.vxMetersPerSecond * mDt,
                    speeds.vyMetersPerSecond * mDt,
                    speeds.omegaRadiansPerSecond * mDt);
            velocity = speeds;
            pose = GeometryUtil.transformBy(pose, GeometryUtil.kPoseZero.exp(twist));
            time += mDt;
        }
        assertEquals(196, pose.getTranslation().getX(), 0.2);
        assertEquals(13, pose.getTranslation().getY(), 0.1);
        assertEquals(0, pose.getRotation().getDegrees(), 1.0);
    }

    @Test
    void testAllTrajectories() {
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(logger);
        DrivePIDFFollower controller = new DrivePIDFFollower(PIDFlog, util, false, 2.4, 2.4);
        TrajectoryGenerator100 generator = new TrajectoryGenerator100();
        List<Trajectory100> trajectories = generator.getTrajectorySet().getAllTrajectories();

        for (var traj : trajectories) {
            TrajectoryTimeIterator traj_iterator = new TrajectoryTimeIterator(
                    new TrajectoryTimeSampler(traj));
            controller.setTrajectory(traj_iterator);
            final Pose2d kInjectedError = new Pose2d(0.3, -0.1, Rotation2d.fromDegrees(9.0));
            final ChassisSpeeds kInjectedVelocityError = new ChassisSpeeds(0.1, 0.3, 0.0);
            final double kInjectionTime = 20.0;
            Pose2d pose = traj.getPoint(0).state().state().getPose();
            ChassisSpeeds velocity = new ChassisSpeeds();
            SwerveSetpoint setpoint = null;
            double time = 0.0;
            double mDt = 0.005;
            boolean error_injected = false;
            while (!controller.isDone()) {
                if (!error_injected && time >= kInjectionTime) {
                    pose = GeometryUtil.transformBy(pose, kInjectedError);
                    velocity = velocity.plus(kInjectedVelocityError);
                    error_injected = true;
                }
                ChassisSpeeds speeds = controller.update(time, pose, velocity);
                if (true) {// setpoint == null) {
                    // Initialize from first chassis speeds.
                    SwerveModuleState100[] states = kSmoothKinematicLimits.toSwerveModuleStates(
                            speeds,
                            velocity.omegaRadiansPerSecond);
                    setpoint = new SwerveSetpoint(speeds, states);
                }

                // Don't use a twist here (assume Drive compensates for that)
                Pose2d delta = new Pose2d(
                        new Translation2d(
                                setpoint.getChassisSpeeds().vxMetersPerSecond * mDt,
                                setpoint.getChassisSpeeds().vyMetersPerSecond * mDt),
                        Rotation2d.fromRadians(setpoint.getChassisSpeeds().omegaRadiansPerSecond * mDt));
                pose = GeometryUtil.transformBy(pose, delta);
                velocity = setpoint.getChassisSpeeds();

                // Inches and degrees
                Pose2d error = GeometryUtil.transformBy(GeometryUtil.inverse(pose),
                        controller.getSetpoint(time).get().state().getPose());

                assertEquals(0.0, error.getTranslation().getX(), 0.0508);
                assertEquals(0.0, error.getTranslation().getY(), 0.0508);
                assertEquals(0.0, error.getRotation().getDegrees(), 5.0);

                time += mDt;
            }
        }
    }
}