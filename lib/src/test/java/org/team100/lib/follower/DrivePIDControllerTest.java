package org.team100.lib.follower;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimedPose;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.TrajectoryTimeIterator;
import org.team100.lib.trajectory.TrajectoryTimeSampler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class DrivePIDControllerTest {
    boolean dump = false;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private static final SwerveKinodynamics kSmoothKinematicLimits = SwerveKinodynamicsFactory.forTest();

    @Test
    void testPIDControl() {
        // first right and then ahead
        List<Pose2d> waypoints = List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(270)),
                new Pose2d(10, -10, Rotation2d.fromDegrees(0)));
        // face +y and end up -x
        List<Rotation2d> headings = List.of(
                GeometryUtil.fromDegrees(90),
                GeometryUtil.fromDegrees(180));
        // so this trajectory is actually (robot-relative) -x the whole way, more or
        // less.

        List<TimingConstraint> constraints = new TimingConstraintFactory(kSmoothKinematicLimits).fast();

        Trajectory100 trajectory = TrajectoryPlanner.restToRest(
                waypoints,
                headings,
                constraints);

        // why is this so large?
        assertEquals(1300, trajectory.length());
        // System.out.println(trajectory);

        TrajectoryTimeSampler view = new TrajectoryTimeSampler(trajectory);

        TrajectoryTimeIterator iter = new TrajectoryTimeIterator(view);
        DriveTrajectoryFollowerUtil util = new DriveTrajectoryFollowerUtil(logger);
        DrivePIDFFollower.Log PIDFlog = new DrivePIDFFollower.Log(logger);
        DrivePIDFFollower controller = new DrivePIDFFollower(PIDFlog, util, false, 2.4, 2.4);
        controller.setTrajectory(iter);

        // this is a series of perfect trajectory following states,
        // based on the trajectory itself.

        {
            ChassisSpeeds output = controller.update(0,
                    new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(1.57079632679)),
                    new ChassisSpeeds());
            verify(0, 0, 0, output);
        }

        {
            Pose2d measurement = new Pose2d(new Translation2d(0.25, -3.5), Rotation2d.fromRadians(1.69));
            ChassisSpeeds output = controller.update(4.0, measurement, new ChassisSpeeds());
            // remember, facing +90, moving -90, so this should be like -1
            // turning slowly to the left
            verify(-0.863, -0.064, 0.064, output);

            TimedPose path_setpoint = controller.getSetpoint(4).get();
            assertEquals(0.24, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-3.5, path_setpoint.state().getPose().getY(), 0.05);
            assertEquals(1.69, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(4, path_setpoint.getTimeS(), 0.01);
            assertEquals(0.979, path_setpoint.velocityM_S(), 0.01);
            assertEquals(-0.008, path_setpoint.acceleration(), 0.001);

            Twist2d errorTwist = DriveTrajectoryFollowerUtil.getErrorTwist(measurement, path_setpoint);
            assertEquals(0, errorTwist.dx, 0.05);
            assertEquals(0, errorTwist.dy, 0.05);
            assertEquals(0, errorTwist.dtheta, 0.05);
        }
        {
            Pose2d measurement = new Pose2d(new Translation2d(1.74, -6.97), Rotation2d.fromRadians(2.22));
            ChassisSpeeds output = controller.update(8.0, measurement, new ChassisSpeeds());
            verify(-0.952, -0.05, 0.098, output);

            TimedPose path_setpoint = controller.getSetpoint(8).get();
            assertEquals(1.74, path_setpoint.state().getPose().getX(), 0.01);
            assertEquals(-6.97, path_setpoint.state().getPose().getY(), 0.01);
            assertEquals(2.19, path_setpoint.state().getHeading().getRadians(), 0.01);
            assertEquals(8, path_setpoint.getTimeS(), 0.001);
            assertEquals(0.955, path_setpoint.velocityM_S(), 0.001);
            assertEquals(0, path_setpoint.acceleration(), 0.001);

            Twist2d errorTwist = DriveTrajectoryFollowerUtil.getErrorTwist(measurement, path_setpoint);
            assertEquals(0.00, errorTwist.dx, 0.01);
            assertEquals(0, errorTwist.dy, 0.01);
            assertEquals(-0.03, errorTwist.dtheta, 0.01);
        }
    }

    void verify(double vx, double vy, double omega, ChassisSpeeds output) {
        assertEquals(vx, output.vxMetersPerSecond, 0.05);
        assertEquals(vy, output.vyMetersPerSecond, 0.05);
        assertEquals(omega, output.omegaRadiansPerSecond, 0.05);
    }
}
