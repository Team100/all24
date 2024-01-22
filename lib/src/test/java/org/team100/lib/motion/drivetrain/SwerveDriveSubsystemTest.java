package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.MockHeading;
import org.team100.lib.testing.TimelessTest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveSubsystemTest extends TimelessTest {

    private static final double kDelta = 0.001;

    Fixture fixture = new Fixture();

    @AfterEach
    void close() {
        fixture.close();
    }

    @Test
    void testSimple() {

        MockHeading heading = new MockHeading();

        Rotation2d gyroAngle = GeometryUtil.kRotationZero;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Pose2d initialPoseMeters = GeometryUtil.kPoseZero;

        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        SwerveDrivePoseEstimator poseEstimator = swerveKinodynamics.newPoseEstimator(
                gyroAngle, modulePositions, initialPoseMeters);

        SwerveModuleCollection modules = fixture.collection;

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, modules);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                heading,
                poseEstimator,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);

        stepTime(0.02);
        drive.periodic();

        drive.driveInFieldCoords(new Twist2d(1, 1, 1), 0.02);

        stepTime(0.02);

        drive.setChassisSpeeds(new ChassisSpeeds(), 0.02);

        stepTime(0.02);
        drive.periodic();

        stepTime(0.02);
        drive.periodic();
        drive.setRawModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        });

        stepTime(0.02);
        drive.periodic();
        drive.defense();

        stepTime(0.02);
        drive.periodic();
        drive.stop();

        drive.close();
    }

    @Test
    void testAccel() {
        Experiments.instance.testOverride(Experiment.UseSetpointGenerator, true);

        MockHeading heading = new MockHeading();

        Rotation2d gyroAngle = GeometryUtil.kRotationZero;
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
        Pose2d initialPoseMeters = GeometryUtil.kPoseZero;

        // this uses max accel of 1, max velocity of 1
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        SwerveDrivePoseEstimator poseEstimator = swerveKinodynamics.newPoseEstimator(
                gyroAngle, modulePositions, initialPoseMeters);

        SwerveModuleCollection modules = fixture.collection;

        SwerveLocal swerveLocal = new SwerveLocal(swerveKinodynamics, modules);

        SwerveDriveSubsystem drive = new SwerveDriveSubsystem(
                heading,
                poseEstimator,
                swerveLocal,
                () -> DriverControl.Speed.NORMAL);
        drive.resetPose(new Pose2d());

        stepTime(0.02);
        drive.periodic();

        // no command, initial state.
        Pose2d x = drive.getPose();
        assertEquals(0, x.getX());
        Twist2d v = drive.getVelocity();
        assertEquals(0, v.dx);
        Twist2d a = drive.getAcceleration();
        assertEquals(0, a.dx);
        SwerveState s = drive.getState();
        assertEquals(0, s.x().x());
        assertEquals(0, s.x().v());
        assertEquals(0, s.x().a());

        // set the motor speed...
        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);
        // this state is hidden...
        assertEquals(0, fixture.collection.states()[0].speedMetersPerSecond);

        stepTime(0.02);
        drive.periodic();

        // until after periodic...
        assertEquals(0.02, fixture.collection.states()[0].speedMetersPerSecond);

        // new state should be accelerating
        x = drive.getPose();
        assertEquals(0.001, x.getX(), kDelta);
        v = drive.getVelocity();
        assertEquals(0.02, v.dx, kDelta);
        a = drive.getAcceleration();
        assertEquals(1, a.dx, kDelta);
        s = drive.getState();
        assertEquals(0.0006, s.x().x(), kDelta);
        assertEquals(0.02, s.x().v(), kDelta);
        assertEquals(1, s.x().a(), kDelta);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);

        stepTime(0.02);
        drive.periodic();

        // keep accelerating
        x = drive.getPose();
        assertEquals(0.0015, x.getX(), kDelta);
        v = drive.getVelocity();
        assertEquals(0.04, v.dx, kDelta);
        a = drive.getAcceleration();
        assertEquals(1, a.dx, kDelta);
        s = drive.getState();
        assertEquals(0.0015, s.x().x(), kDelta);
        assertEquals(0.04, s.x().v(), kDelta);
        assertEquals(1, s.x().a(), kDelta);

        drive.setChassisSpeeds(new ChassisSpeeds(1, 0, 0), 0.02);

        stepTime(0.02);
        drive.periodic();

        // keep accelerating
        x = drive.getPose();
        assertEquals(0.002, x.getX(), kDelta);
        v = drive.getVelocity();
        assertEquals(0.06, v.dx, kDelta);
        a = drive.getAcceleration();
        assertEquals(1, a.dx, kDelta);
        s = drive.getState();
        assertEquals(0.002, s.x().x(), kDelta);
        assertEquals(0.06, s.x().v(), kDelta);
        assertEquals(1, s.x().a(), kDelta);

        drive.close();
    }
}
