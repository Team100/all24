package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveDynamicsConstraintTest {
    private static final double kDelta = 0.001;
    // the free speed of a module, which is also the free speed
    // of the robot going in a straight line without rotating.
    private static final double maxV = 4;
    private static final SwerveDriveKinematics k = new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5));

    @Test
    void testSimple() {
        SwerveKinematicLimits l = new SwerveKinematicLimits(maxV, 2, 10);
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(k, l);

        // motionless
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
        assertEquals(0.929, m, kDelta);
    }

    @Test
    void testDesaturation() {
        // this is for comparison to the above case.

        // treat the drivetrain as a 1m circle rolling on its edge.
        // rotational speed in rad/s is double translation speed.
        // since it's a square, the numbers aren't the same.

        // start with too-fast speed.  this is 5 rad/m, as above,
        // with 1 m/s linear speed, so also 5 rad/s.
        ChassisSpeeds s = new ChassisSpeeds(1, 0, 5);
        SwerveModuleState[] ms = k.toSwerveModuleStates(s);
        assertEquals(2.915, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(4.301, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(2.915, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(4.301, ms[3].speedMetersPerSecond, kDelta);

        // this is slowed to the max possible wheel speed
        SwerveDriveKinematics.desaturateWheelSpeeds(ms, maxV);
        assertEquals(2.711, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(4, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(2.711, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(4, ms[3].speedMetersPerSecond, kDelta);

        // the resulting chassis speeds.  This maintains 5 rad/m
        // and slows down to achieve it.
        ChassisSpeeds implied = k.toChassisSpeeds(ms);
        assertEquals(0.929, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(4.649, implied.omegaRadiansPerSecond, kDelta);
    }

        @Test
    void testDesaturation2() {
        // 0.62 m/s is pretty close to the maximum speed
        // possible at 5 rad/s; this is about 8 rad/m.
        ChassisSpeeds s = new ChassisSpeeds(0.62, 0, 5);
        SwerveModuleState[] ms = k.toSwerveModuleStates(s);
        SwerveDriveKinematics.desaturateWheelSpeeds(ms, maxV);

        ChassisSpeeds implied = k.toChassisSpeeds(ms);
        assertEquals(0.62, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(5, implied.omegaRadiansPerSecond, kDelta);
    }

}
