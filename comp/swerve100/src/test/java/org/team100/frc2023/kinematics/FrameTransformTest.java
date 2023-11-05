package org.team100.frc2023.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.VeeringCorrection;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FrameTransformTest {
    private static final double kDelta = 0.01;

    /** This is identical to the test in WPI's ChassisSpeedsTest. */
    @Test
    void testFieldRelativeConstruction() {
        FrameTransform factory = new FrameTransform();
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(1.0, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCounterclockwiseRotationCorrection() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 1.0));
        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );
        // correction is +x-in-robot-frame, (-y-in-field-frame) to counteract the
        // lag-induced error, +y-in-field-frame
        assertEquals(0.149, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(0.98, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testClockwiseRotationCorrection() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> -1.0));
        final var chassisSpeeds = factory.fromFieldRelativeSpeeds(
                1.0, // vx
                0.0, // vy
                0.5, // omega
                Rotation2d.fromDegrees(-90.0) // robot angle
        );

        // correction is -x-in-robot-frame, (+y-in-field-frame) to counteract the
        // lag-induced error, -y-in-field-frame
        assertEquals(-0.149, chassisSpeeds.vxMetersPerSecond, kDelta); // sin 0.2
        assertEquals(0.98, chassisSpeeds.vyMetersPerSecond, kDelta); // cos 0.2
        assertEquals(0.5, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCounterclockwise2() {
        // rotation counter clockwise is currently *negative*
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 1.0));
        // driving straight ahead
        ChassisSpeeds chassisSpeeds = factory.fromFieldRelativeSpeeds(1.0, 0.0, 0.0, Rotation2d.fromDegrees(0));
        // will tend to veer to the left, so the correction would be towards the right
        assertEquals(0.98, chassisSpeeds.vxMetersPerSecond, kDelta);
        assertEquals(-0.149, chassisSpeeds.vyMetersPerSecond, kDelta);
        assertEquals(0, chassisSpeeds.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCorrectionCounterclockwise() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 1.0));
        Rotation2d angle = factory.correctAngle(Rotation2d.fromDegrees(-90.0));
        assertEquals(-81.4, angle.getDegrees(), kDelta);
    }

    @Test
    void testInverse0() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 0.0));
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(0, 0, 0, new Rotation2d());
        assertEquals(0, fieldRelative.dx, kDelta);
        assertEquals(0, fieldRelative.dy, kDelta);
        assertEquals(0, fieldRelative.dtheta, kDelta);
    }

    @Test
    void testInverse45() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 0.0));
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(1, 0, 0, new Rotation2d(Math.PI / 4));
        assertEquals(Math.sqrt(2) / 2, fieldRelative.dx, kDelta);
        assertEquals(Math.sqrt(2) / 2, fieldRelative.dy, kDelta);
        assertEquals(0, fieldRelative.dtheta, kDelta);
    }

    @Test
    void testInverse135Rot() {
        FrameTransform factory = new FrameTransform(new VeeringCorrection(() -> 0.0));
        Twist2d fieldRelative = factory.toFieldRelativeSpeeds(0, 1, 1, new Rotation2d(3 * Math.PI / 4));
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.dx, kDelta);
        assertEquals(-1.0 * Math.sqrt(2) / 2, fieldRelative.dy, kDelta);
        assertEquals(1, fieldRelative.dtheta, kDelta);
    }
}
