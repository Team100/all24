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

    @Test
    void testSimple() {
        // the free speed of a module, which is also the free speed
        // of the robot going in a straight line without rotating.
        final double maxV = 4;
        SwerveDriveKinematics k = new SwerveDriveKinematics(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveKinematicLimits l = new SwerveKinematicLimits(maxV, 2, 10);
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(k, l);

        // motionless
        double m = c.getMaxVelocity(new Pose2dWithMotion());
        assertEquals(4, m, kDelta);

        // moving in +x, no curvature
        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(1, 0, 0),
                0, 0));
        assertEquals(4, m, kDelta);

        // treat the drivetrain as a 1m circle rolling on its edge.
        // rotational speed in rad/s is double translation speed.
        // since it's a square, the numbers aren't the same.

        // start with too-fast speed
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

        // same v/omega ratio
        ChassisSpeeds implied = k.toChassisSpeeds(ms);
        assertEquals(0.929, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(4.649, implied.omegaRadiansPerSecond, kDelta);

        m = c.getMaxVelocity(new Pose2dWithMotion(
                new Pose2d(),
                new Twist2d(2, 0, 5),  // the "2" here is treated as "1"
                0, 0));
        // this is max possible wheel speed / max actual wheel speed
        // so it's like a ratio?
        assertEquals(0.929, m, kDelta);

        // this doesn't really make any sense. I *think* it's trying to do the same
        // thing as the desaturator.
    }

}
