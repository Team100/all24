

package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveControlTest {
    private static final double kDelta = 0.001;

    @Test
    void testTransform() {
        Pose2d p = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));
        FieldRelativeVelocity t = new FieldRelativeVelocity(1, 1, 1);
        SwerveControl s = new SwerveControl(p, t);
        assertEquals(1, s.x().x(), kDelta);
    }

    @Test
    void testTimedPose() {
        SwerveControl s = SwerveControl.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new Twist2d(0, 0, 0), 0, 0),
                        0, 0, 0));
        assertEquals(0, s.x().x(), kDelta);
        assertEquals(0, s.x().v(), kDelta);
        assertEquals(0, s.x().a(), kDelta);
        assertEquals(0, s.y().x(), kDelta);
        assertEquals(0, s.y().v(), kDelta);
        assertEquals(0, s.y().a(), kDelta);
    }

    @Test
    void testTimedPose2() {
        SwerveControl s = SwerveControl.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new Twist2d(0, 0, 0), 0, 0),
                        0, 0, 1));
        assertEquals(0, s.x().x(), kDelta);
        assertEquals(0, s.x().v(), kDelta);
        assertEquals(1, s.x().a(), kDelta);
        assertEquals(0, s.y().x(), kDelta);
        assertEquals(0, s.y().v(), kDelta);
        assertEquals(0, s.y().a(), kDelta);
    }

    @Test
    void testTimedPose3() {
        SwerveControl s = SwerveControl.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new Twist2d(1, 0, 0), 0, 0),
                        0, 1, 0));
        assertEquals(0, s.x().x(), kDelta);
        assertEquals(1, s.x().v(), kDelta);
        assertEquals(0, s.x().a(), kDelta);
        assertEquals(0, s.y().x(), kDelta);
        assertEquals(0, s.y().v(), kDelta);
        assertEquals(0, s.y().a(), kDelta);
    }

    /** +x motion, positive curvature => +y accel. */
    @Test
    void testTimedPose4() {
        SwerveControl s = SwerveControl.fromTimedPose(
                new TimedPose(
                        new Pose2dWithMotion(
                                new Pose2d(0, 0, new Rotation2d()),
                                new Twist2d(1, 0, 0), 1, 0),
                        0, 1, 0));
        assertEquals(0, s.x().x(), kDelta);
        assertEquals(1, s.x().v(), kDelta);
        assertEquals(0, s.x().a(), kDelta);
        assertEquals(0, s.y().x(), kDelta);
        assertEquals(0, s.y().v(), kDelta);
        assertEquals(1, s.y().a(), kDelta);
    }

    @Test
    void testChassisSpeeds0() {
        SwerveControl state = new SwerveControl(
                new Pose2d(new Translation2d(0, 0), GeometryUtil.kRotation180),
                new FieldRelativeVelocity(1, 0, 0));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(-1, speeds.vxMetersPerSecond, kDelta);
        assertEquals(0, speeds.vyMetersPerSecond, kDelta);
        assertEquals(0, speeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testChassisSpeeds1() {
        SwerveControl state = new SwerveControl(
                new Pose2d(new Translation2d(0, 0), GeometryUtil.kRotation90),
                new FieldRelativeVelocity(1, 0, 1));
        ChassisSpeeds speeds = state.chassisSpeeds();
        assertEquals(0, speeds.vxMetersPerSecond, kDelta);
        assertEquals(-1, speeds.vyMetersPerSecond, kDelta);
        assertEquals(1, speeds.omegaRadiansPerSecond, kDelta);
    }
}
