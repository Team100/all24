package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.swerve.AsymSwerveSetpointGenerator;
import org.team100.lib.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

class SwerveSetpointGeneratorConstraintTest {
    private static final double kDelta = 0.001;
    private static final SwerveDriveKinematics k = new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    @Test
    void testSpin() {
        AsymSwerveSetpointGenerator.KinematicLimits l = new AsymSwerveSetpointGenerator.KinematicLimits(4, 2, 5, 10);
        SwerveSetpointGeneratorConstraint c = new SwerveSetpointGeneratorConstraint(k, l);

        double m = c.getMaxVelocity(Pose2dWithMotion.kIdentity);
        assertEquals(4, m, kDelta);

        // moving in +x, no curvature, no rotation
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 0),
                0, 0));
        // max allowed velocity is full speed
        assertEquals(4, m, kDelta);

        // moving in +x, 5 rad/meter
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 5),
                0, 0));
        // at 5 rad/m the fastest you can go is 0.929 m/s.
        // the setpoint generator is a bit more conservative.
        // TODO: is this wrong?
        assertEquals(0.855, m, kDelta);
    }

    // enforce centripetal acceleration limit?
    @Test
    void testCurve() {
        AsymSwerveSetpointGenerator.KinematicLimits l = new AsymSwerveSetpointGenerator.KinematicLimits(4, 2, 5, 10);
        SwerveSetpointGeneratorConstraint c = new SwerveSetpointGeneratorConstraint(k, l);

        double m = c.getMaxVelocity(Pose2dWithMotion.kIdentity);
        assertEquals(4, m, kDelta);

        // moving in +x, curve to the left
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 0),
                1, 0));
        // max allowed velocity is full speed, which is wrong
        // FIXME
        assertEquals(4, m, kDelta);

        // moving in +x, very sharp corner
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 0),
                10, 0));

        // max allowed velocity is full speed, which is wrong
        // FIXME
        assertEquals(4, m, kDelta);
    }

    @Test
    void testAccel() {
        AsymSwerveSetpointGenerator.KinematicLimits l = new AsymSwerveSetpointGenerator.KinematicLimits(4, 2, 5, 10);
        SwerveSetpointGeneratorConstraint c = new SwerveSetpointGeneratorConstraint(k, l);
        // this is constant
        MinMaxAcceleration m = c.getMinMaxAcceleration(Pose2dWithMotion.kIdentity, 0);
        assertEquals(-5, m.getMinAccel(), kDelta);
        assertEquals(2, m.getMaxAccel(), kDelta);
    }

}
