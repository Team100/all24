package org.team100.lib.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.timing.TimingConstraint.MinMaxAcceleration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

class SwerveDriveDynamicsConstraintTest {
    private static final double kDelta = 0.001;
    // the free speed of a module, which is also the free speed
    // of the robot going in a straight line without rotating.
    private static final double maxV = 4;

    @Test
    void testVelocity() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(l);

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
        // at 5 rad/m with 0.5m sides the fastest you can go is 1.55 m/s.
        assertEquals(1.509, m, kDelta);
    }

    @Test
    void testAccel() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        SwerveDriveDynamicsConstraint c = new SwerveDriveDynamicsConstraint(l);
        // this is constant
        MinMaxAcceleration m = c.getMinMaxAcceleration(Pose2dWithMotion.kIdentity, 0);
        assertEquals(-4, m.getMinAccel(), kDelta);
        assertEquals(4, m.getMaxAccel(), kDelta);
    }

    @Test
    void testDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        // this is for comparison to the above case.

        // treat the drivetrain as a 1m circle rolling on its edge.
        // rotational speed in rad/s is double translation speed.
        // since it's a square, the numbers aren't the same.

        // start with too-fast speed.
        ChassisSpeeds s = new ChassisSpeeds(1, 0, 10);
        SwerveModuleState[] ms = l.toSwerveModuleStates(s, 10, 0.02);
        assertEquals(2.661, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(4.061, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(3.243, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(4.464, ms[3].speedMetersPerSecond, kDelta);

        // this is slowed to the max possible wheel speed
        SwerveDriveKinematics.desaturateWheelSpeeds(ms, maxV);
        assertEquals(2.384, ms[0].speedMetersPerSecond, kDelta);
        assertEquals(3.639, ms[1].speedMetersPerSecond, kDelta);
        assertEquals(2.906, ms[2].speedMetersPerSecond, kDelta);
        assertEquals(4, ms[3].speedMetersPerSecond, kDelta);

        // the resulting chassis speeds. This slows to try to
        // to maintain the rotational speed
        ChassisSpeeds implied = l.toChassisSpeeds(ms);
        assertEquals(0.843, implied.vxMetersPerSecond, kDelta);
        assertEquals(-0.308, implied.vyMetersPerSecond, kDelta);
        assertEquals(8.961, implied.omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testDesaturation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        // 0.62 m/s is pretty close to the maximum speed
        // possible at 5 rad/s; this is about 8 rad/m.
        ChassisSpeeds s = new ChassisSpeeds(0.62, 0, 5);
        SwerveModuleState[] ms = l.toSwerveModuleStates(s, 5, 0.02);
        SwerveDriveKinematics.desaturateWheelSpeeds(ms, maxV);

        ChassisSpeeds implied = l.toChassisSpeeds(ms);
        assertEquals(0.611, implied.vxMetersPerSecond, kDelta);
        assertEquals(-0.108, implied.vyMetersPerSecond, kDelta);
        assertEquals(5, implied.omegaRadiansPerSecond, kDelta);
    }

  

}
