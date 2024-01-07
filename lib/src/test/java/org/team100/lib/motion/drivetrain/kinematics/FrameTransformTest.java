package org.team100.lib.motion.drivetrain.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class FrameTransformTest {
    private static final double kDelta = 0.01;

    /** This is identical to the test in WPI's ChassisSpeedsTest. */
    @Test
    void testFieldRelativeConstruction() {
        FrameTransform factory = new FrameTransform();
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                0,
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(1.0, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCounterclockwiseRotationCorrection() {
        FrameTransform factory = new FrameTransform();
        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                1,
                Rotation2d.fromDegrees(-90.0) // robot angle
        );
        // correction is +x-in-robot-frame, (-y-in-field-frame) to counteract the
        // lag-induced error, +y-in-field-frame
        assertEquals(0.049, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(0.999, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testClockwiseRotationCorrection() {
        FrameTransform factory = new FrameTransform();
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                -1,
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        // correction is -x-in-robot-frame, (+y-in-field-frame) to counteract the
        // lag-induced error, -y-in-field-frame
        assertEquals(-0.049, chassisSpeeds.vxMetersPerSecond, kDelta); // sin 0.2
        assertEquals(0.999, chassisSpeeds.vyMetersPerSecond, kDelta); // cos 0.2
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCounterclockwise2() {
        // rotation counter clockwise is currently *negative*
        FrameTransform factory = new FrameTransform();
        // driving straight ahead
        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(1.0, 0.0, 0.0, 1, Rotation2d.fromDegrees(0));
        // will tend to veer to the left, so the correction would be towards the right
        assertEquals(0.999, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(-0.049, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCorrectionCounterclockwise() {
        FrameTransform factory = new FrameTransform();
        Rotation2d angle = factory.correctAngle(1, Rotation2d.fromDegrees(-90.0));
        assertEquals(-87.135, angle.getDegrees(), kDelta);
    }

    @Test
    void testInverse0() {
        FrameTransform factory = new FrameTransform();
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(0, 0, 0, 0, GeometryUtil.kRotationZero);
        assertEquals(0, fieldRelative.dx, kDelta);
        assertEquals(0, fieldRelative.dy, kDelta);
        assertEquals(0, fieldRelative.dtheta, kDelta);
    }

    @Test
    void testInverse45() {
        FrameTransform factory = new FrameTransform();
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(1, 0, 0, 0, new Rotation2d(Math.PI / 4));
        assertEquals(Math.sqrt(2) / 2, fieldRelative.dx, kDelta);
        assertEquals(Math.sqrt(2) / 2, fieldRelative.dy, kDelta);
        assertEquals(0, fieldRelative.dtheta, kDelta);
    }

    @Test
    void testInverse135Rot() {
        FrameTransform factory = new FrameTransform();
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(0, 1, 1, 0, new Rotation2d(3 * Math.PI / 4));
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.dx, kDelta);
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.dy, kDelta);
        assertEquals(1, fieldRelative.dtheta, kDelta);
    }

    @Test
    void testRoundTrip() {
        // field relative
        final double dx = 1;
        final double dy = 0;
        final double dtheta = 0;
        final double theta = 0;

        FrameTransform factory = new FrameTransform();

        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(
                dx, dy, dtheta, 0, Rotation2d.fromDegrees(theta));
        assertEquals(1, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(0, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0, chassisSpeeds.omegaRadiansPerSecond, kDelta);

        Twist2d twist = factory.toFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, 
                chassisSpeeds.omegaRadiansPerSecond,
                0,
                Rotation2d.fromDegrees(theta));
        assertEquals(1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testRoundTripNoVeering() {

        // field relative
        final double dx = 1;
        final double dy = 0;
        final double dtheta = 1; // now spinning
        final double theta = 0;

        // no veering correction
        FrameTransform factory = new FrameTransform();

        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(
                dx, dy, dtheta, 0, Rotation2d.fromDegrees(theta));
        assertEquals(1, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(0, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(1, chassisSpeeds.omegaRadiansPerSecond, kDelta);

        Twist2d twist = factory.toFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, 
                chassisSpeeds.omegaRadiansPerSecond,
                0,
                Rotation2d.fromDegrees(theta));
        assertEquals(1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(1, twist.dtheta, kDelta);
    }

    @Test
    void testRoundTripWithVeering() {
        // field relative
        final double dx = 1;
        final double dy = 0;
        final double dtheta = 1; // now spinning
        final double theta = 0;

        // now with veering correction, same spin as above
        FrameTransform factory = new FrameTransform();

        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(
                dx, dy, dtheta, dtheta, Rotation2d.fromDegrees(theta));
        assertEquals(0.989, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(-0.049, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(1, chassisSpeeds.omegaRadiansPerSecond, kDelta);

        Twist2d twist = factory.toFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond, 
                chassisSpeeds.omegaRadiansPerSecond,
                dtheta,
                Rotation2d.fromDegrees(theta));
        
        // this should correct the other way.
        assertEquals(1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(1, twist.dtheta, kDelta);
    }
}
