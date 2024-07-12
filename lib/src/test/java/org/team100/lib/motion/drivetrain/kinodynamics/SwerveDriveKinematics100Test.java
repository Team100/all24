package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * None of these tests take tires into account.
 */
class SwerveDriveKinematics100Test {
    private static final double kDelta = 0.001;

    @Test
    void testTwistStraight() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.toTwist2d(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))));

        assertEquals(0.1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testTwistSpin() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        Twist2d twist = kinematics.toTwist2d(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(135))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(45))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(-135))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(-45))));

        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0.141, twist.dtheta, kDelta);
    }

    @Test
    void testWithTime() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.toTwist2d(
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModulePosition100(0.1, Optional.of(Rotation2d.fromDegrees(0))));

        assertEquals(0.1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    private static final double kEpsilon = 1E-9;

    private final Translation2d m_fl = new Translation2d(12, 12);
    private final Translation2d m_fr = new Translation2d(12, -12);
    private final Translation2d m_bl = new Translation2d(-12, 12);
    private final Translation2d m_br = new Translation2d(-12, -12);

    private final SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(m_fl, m_fr, m_bl, m_br);

    @Test
    void testStraightLineInverseKinematics() { // test inverse kinematics going in a straight line

        ChassisSpeeds speeds = new ChassisSpeeds(5, 0, 0);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        assertAll(
                () -> assertEquals(5.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[0].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].angle.get().getRadians(), kEpsilon));
    }

    @Test
    void testStraightLineForwardKinematics() { // test forward kinematics going in a straight line
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        // does not take tires into account
        var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

        assertAll(
                () -> assertEquals(5.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testStraightLineForwardKinematicsWithDeltas() {
        // test forward kinematics going in a straight line
        SwerveModulePosition100 delta = new SwerveModulePosition100(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        var twist = m_kinematics.toTwist2d(delta, delta, delta, delta);

        assertAll(
                () -> assertEquals(5.0, twist.dx, kEpsilon),
                () -> assertEquals(0.0, twist.dy, kEpsilon),
                () -> assertEquals(0.0, twist.dtheta, kEpsilon));
    }

    @Test
    void testStraightStrafeInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 5, 0);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        assertAll(
                () -> assertEquals(5.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(90.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testStraightStrafeForwardKinematics() {
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(90.0)));
        // does not take tires into account
        var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testStraightStrafeForwardKinematicsWithDeltas() {
        SwerveModulePosition100 delta = new SwerveModulePosition100(5.0, Optional.of(Rotation2d.fromDegrees(90.0)));
        var twist = m_kinematics.toTwist2d(delta, delta, delta, delta);

        assertAll(
                () -> assertEquals(0.0, twist.dx, kEpsilon),
                () -> assertEquals(5.0, twist.dy, kEpsilon),
                () -> assertEquals(0.0, twist.dtheta, kEpsilon));
    }

    @Test
    void testConserveWheelAngle() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        m_kinematics.toSwerveModuleStates(speeds);
        var moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());

        // Robot is stationary, but module angles are preserved.

        assertAll(
                () -> assertEquals(0.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(135.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(45.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-135.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testResetWheelAngle() {
        Rotation2d fl = new Rotation2d(0);
        Rotation2d fr = new Rotation2d(Math.PI / 2);
        Rotation2d bl = new Rotation2d(Math.PI);
        Rotation2d br = new Rotation2d(3 * Math.PI / 2);
        m_kinematics.resetHeadings(fl, fr, bl, br);
        var moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());

        // Robot is stationary, but module angles are preserved.

        assertAll(
                () -> assertEquals(0.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(180.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(270.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testTurnInPlaceInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        /*
         * The circumference of the wheels about the COR is π * diameter, or 2π * radius
         * the radius is the √(12²in + 12²in), or 16.9706in, so the circumference the
         * wheels
         * trace out is 106.629190516in. since we want our robot to rotate at 1 rotation
         * per second,
         * our wheels must trace out 1 rotation (or 106.63 inches) per second.
         */

        assertAll(
                () -> assertEquals(106.63, moduleStates[0].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[1].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[2].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[3].speedMetersPerSecond, 0.1),
                () -> assertEquals(135.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(45.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-135.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testTurnInPlaceForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModuleState100 frState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModuleState100 blState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModuleState100 brState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-45)));
        // does not take tires into account
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omegaRadiansPerSecond, 0.1));
    }

    @Test
    void testTurnInPlaceForwardKinematicsWithDeltas() {
        SwerveModulePosition100 flDelta = new SwerveModulePosition100(106.629,
                Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModulePosition100 frDelta = new SwerveModulePosition100(106.629, Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModulePosition100 blDelta = new SwerveModulePosition100(106.629,
                Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModulePosition100 brDelta = new SwerveModulePosition100(106.629,
                Optional.of(Rotation2d.fromDegrees(-45)));

        var twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        assertAll(
                () -> assertEquals(0.0, twist.dx, kEpsilon),
                () -> assertEquals(0.0, twist.dy, kEpsilon),
                () -> assertEquals(2 * Math.PI, twist.dtheta, 0.1));
    }

    @Test
    void testOffCenterCORRotationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(0.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 frState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 blState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModuleState100 brState = new SwerveModuleState100(213.258, Optional.of(Rotation2d.fromDegrees(-45)));
        // does not take tires into account
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, chassisSpeeds.vxMetersPerSecond, 0.1),
                () -> assertEquals(-75.398, chassisSpeeds.vyMetersPerSecond, 0.1),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omegaRadiansPerSecond, 0.1));
    }

    @Test
    void testOffCenterCORRotationForwardKinematicsWithDeltas() {
        SwerveModulePosition100 flDelta = new SwerveModulePosition100(0.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModulePosition100 frDelta = new SwerveModulePosition100(150.796,
                Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModulePosition100 blDelta = new SwerveModulePosition100(150.796,
                Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModulePosition100 brDelta = new SwerveModulePosition100(213.258,
                Optional.of(Rotation2d.fromDegrees(-45)));

        var twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, twist.dx, 0.1),
                () -> assertEquals(-75.398, twist.dy, 0.1),
                () -> assertEquals(2 * Math.PI, twist.dtheta, 0.1));
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModuleState100 frState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModuleState100 blState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModuleState100 brState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-70.56)));
        // does not take tires into account
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, 0.1);
        assertEquals(-33.0, chassisSpeeds.vyMetersPerSecond, 0.1);
        assertEquals(1.5, chassisSpeeds.omegaRadiansPerSecond, 0.1);
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematicsWithDeltas() {
        SwerveModulePosition100 flDelta = new SwerveModulePosition100(23.43,
                Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModulePosition100 frDelta = new SwerveModulePosition100(23.43,
                Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModulePosition100 blDelta = new SwerveModulePosition100(54.08,
                Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModulePosition100 brDelta = new SwerveModulePosition100(54.08,
                Optional.of(Rotation2d.fromDegrees(-70.56)));

        var twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertAll(
                () -> assertEquals(0.0, twist.dx, 0.1),
                () -> assertEquals(-33.0, twist.dy, 0.1),
                () -> assertEquals(1.5, twist.dtheta, 0.1));
    }

    @Test
    void testModuleKinematics() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SimpleMatrix accelerations = new SimpleMatrix(3, 1);
        SimpleMatrix expected = new SimpleMatrix(2, 1);
        accelerations.setColumn(0, 0, 0, 0, 0);
        expected.setColumn(0, 0, 0, 0);
        SimpleMatrix output = kinematics.getModuleAccelerationXY(0, accelerations);
        Util.println(output.get(0, 0) + " " + output.get(1, 0));
        assertEquals(expected.get(0, 0), 0, 0.0001);
        assertEquals(expected.get(1, 0), 0, 0.0001);
    }

    @Test
    void testModuleKinematics2() {
        // one-meter drive base
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        SwerveModuleState100[] prev = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 0, 0, 1);
            SwerveModuleState100[] output2 = kinematics.statesFromVector(v);

            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 0, 0, 0);
            SwerveModuleState100[] output = kinematics.accelerationFromVector(v, a, prev, 0.02);

            Util.println(output[0].toString());

            assertEquals(0, output[0].accelMetersPerSecond_2, 0.0001);
            for (int i = 0; i < output.length; i++) {
                assertEquals(output[i].angle, output2[i].angle);
                assertEquals(output[i].speedMetersPerSecond, output2[i].speedMetersPerSecond);
            }
        }

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 2, 1, 1);
            SwerveModuleState100[] output4 = kinematics.statesFromVector(v);
            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 2, 2, 2);
            SwerveModuleState100[] output3 = kinematics.accelerationFromVector(v, a, prev, 0.02);
            for (int i = 0; i < output3.length; i++) {
                assertEquals(output3[i].angle, output4[i].angle);
                assertEquals(output3[i].speedMetersPerSecond, output4[i].speedMetersPerSecond);
            }
        }

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 2.3, 0, 1.1);

            SwerveModuleState100[] output5 = kinematics.statesFromVector(v);

            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 1.32, 1.2, 2.2);

            SwerveModuleState100[] output6 = kinematics.accelerationFromVector(v, a, prev, 0.02);

            for (int i = 0; i < output5.length; i++) {
                assertEquals(output5[i].angle, output6[i].angle);
                assertEquals(output5[i].speedMetersPerSecond, output6[i].speedMetersPerSecond);
            }
        }

        {
            // motionless
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 0, 0, 0);
            SwerveModuleState100[] output7 = kinematics.statesFromVector(v);
            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 1, 1.1, 2);
            SwerveModuleState100[] output8 = kinematics.accelerationFromVector(v, a, prev, 0.02);
            for (int i = 0; i < output7.length; i++) {
                assertEquals(output7[i].angle, output8[i].angle);
                assertEquals(output7[i].speedMetersPerSecond, output8[i].speedMetersPerSecond);
            }
        }
    }

    @Test
    void testModuleKinematics3a() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        // no velocity
        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        velocities.setColumn(0, 0, 0, 0, 0);

        // accelerating in x i guess
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        acceleration.setColumn(0, 0, 1, 0, 0);

        SwerveModuleState100[] output = kinematics.accelerationFromVector(
                velocities,
                acceleration,
                prevStates,
                0.02);

        for (SwerveModuleState100 state : output) {
            // assertEquals(state.angle.get().getRadians(), 0, 0.0001);
            // since the module isn't yet moving it has an indeterminate angle.
            assertTrue(state.angle.isEmpty());
            assertEquals(state.omega, 0, 0.0001);
            assertEquals(state.speedMetersPerSecond, 0, 0.0001);
            // TODO: fix this test
            // assertEquals(state.accelMetersPerSecond_2, 1, 0.0001);
        }

    }

    @Test
    void testModuleKinematics3b() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        SwerveModuleState100[] output = new SwerveModuleState100[4];

        velocities.setColumn(0, 0, 1, 0, 0);
        acceleration.setColumn(0, 0, 1, 0, 0);

        output = kinematics.accelerationFromVector(velocities,
                acceleration, prevStates, 0.02);

        for (SwerveModuleState100 state : output) {
            assertEquals(state.angle.get().getRadians(), 0, 0.0001);
            assertEquals(state.omega, 0, 0.0001);
            assertEquals(state.speedMetersPerSecond, 1, 0.0001);
            assertEquals(state.accelMetersPerSecond_2, 1, 0.0001);
        }

    }

    @Test
    void testModuleKinematics3c() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        SwerveModuleState100[] output = new SwerveModuleState100[4];

        velocities.setColumn(0, 0, 1, 0, 0);
        acceleration.setColumn(0, 0, 0, 0, 1);
        output = kinematics.accelerationFromVector(velocities,
                acceleration, prevStates, 0.02);

        int count = 0;
        for (SwerveModuleState100 state : output) {
            assertEquals(state.angle.get().getRadians(), 0, 0.0001);
            assertEquals(state.speedMetersPerSecond, 1, 0.0001);
            switch (count) {
                case 0:
                    assertEquals(state.omega, 0.5, 0.0001);
                    assertEquals(state.accelMetersPerSecond_2, -0.5, 0.0001);
                    break;
                case 1:
                    assertEquals(state.omega, 0.5, 0.0001);
                    assertEquals(state.accelMetersPerSecond_2, 0.5, 0.0001);
                    break;
                case 2:
                    assertEquals(state.omega, -0.5, 0.0001);
                    assertEquals(state.accelMetersPerSecond_2, -0.5, 0.0001);
                    break;
                case 3:
                    assertEquals(state.omega, -0.5, 0.0001);
                    assertEquals(state.accelMetersPerSecond_2, 0.5, 0.0001);
                    break;
                default:
                    throw new UnsupportedOperationException("Not a swerve module");
            }
            count++;
        }

    }

    @Test
    void testDesaturate() {
        SwerveModuleState100 fl = new SwerveModuleState100(5, Optional.of(new Rotation2d()));
        SwerveModuleState100 fr = new SwerveModuleState100(6, Optional.of(new Rotation2d()));
        SwerveModuleState100 bl = new SwerveModuleState100(4, Optional.of(new Rotation2d()));
        SwerveModuleState100 br = new SwerveModuleState100(7, Optional.of(new Rotation2d()));

        SwerveModuleState100[] arr = { fl, fr, bl, br };
        SwerveDriveKinematics100.desaturateWheelSpeeds(arr, 5.5);

        double factor = 5.5 / 7.0;

        assertAll(
                () -> assertEquals(5.0 * factor, arr[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(6.0 * factor, arr[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(4.0 * factor, arr[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(7.0 * factor, arr[3].speedMetersPerSecond, kEpsilon));
    }

    @Test
    void testDesaturateNegativeSpeed() {
        SwerveModuleState100 fl = new SwerveModuleState100(1, Optional.of(new Rotation2d()));
        SwerveModuleState100 fr = new SwerveModuleState100(1, Optional.of(new Rotation2d()));
        SwerveModuleState100 bl = new SwerveModuleState100(-2, Optional.of(new Rotation2d()));
        SwerveModuleState100 br = new SwerveModuleState100(-2, Optional.of(new Rotation2d()));

        SwerveModuleState100[] arr = { fl, fr, bl, br };
        SwerveDriveKinematics100.desaturateWheelSpeeds(arr, 1);

        assertAll(
                () -> assertEquals(0.5, arr[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.5, arr[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-1.0, arr[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-1.0, arr[3].speedMetersPerSecond, kEpsilon));
    }
}
