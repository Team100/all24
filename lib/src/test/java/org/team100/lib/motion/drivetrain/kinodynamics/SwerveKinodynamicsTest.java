package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Tire;
import org.team100.lib.util.Util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * None of the tests here involve the Tire model.
 */
class SwerveKinodynamicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testComputedValues() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2,
                1,Tire.noslip());
        assertEquals(1, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omega = driveV / r;
        assertEquals(2.828, omega, kDelta);
        assertEquals(2.828, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedValues2() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2,
                1,Tire.noslip());
        assertEquals(4, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omega = driveV / r;
        assertEquals(11.313, omega, kDelta);
        assertEquals(11.313, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedValues3() {
        double track = 1;
        double wheelbase = 1;
        double driveV = 4;
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2,
                1,Tire.noslip());
        assertEquals(4, k.getMaxDriveVelocityM_S(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, kDelta);

        double omega = driveV / r;
        assertEquals(5.656, omega, kDelta);
        assertEquals(5.656, k.getMaxAngleSpeedRad_S(), kDelta);
    }

    @Test
    void testComputedAngularAcceleration() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(1, 1, driveA, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2,
                1,Tire.noslip());
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.353, r, kDelta);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        assertEquals(8.485, omegaDot, kDelta);
        assertEquals(8.485, k.getMaxAngleAccelRad_S2(), kDelta);
    }

    @Test
    void testComputedAngularAcceleration2() {
        double track = 1;
        double wheelbase = 1;
        double driveA = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(1, 1, driveA, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2,
                1,Tire.noslip());
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double r = Math.hypot(track / 2, wheelbase / 2);
        assertEquals(0.707, r, kDelta);

        double omegaDot = 12 * driveA * r / (track * track + wheelbase * wheelbase);

        // scales inverse with size
        assertEquals(4.242, omegaDot, kDelta);
        assertEquals(4.242, k.getMaxAngleAccelRad_S2(), kDelta);
    }

    @Test
    void testComputedCapsize() {
        double track = 1;
        double wheelbase = 1;
        double vcg = 0.3;
        SwerveKinodynamics k = new SwerveKinodynamics(1, 1, 1, 1, 1, 20 * Math.PI, track, wheelbase, wheelbase / 2, vcg,Tire.noslip());
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double fulcrum = Math.min(track / 2, wheelbase / 2);
        assertEquals(0.5, fulcrum, kDelta);

        double accel = 9.8 * fulcrum / vcg;

        assertEquals(16.333, accel, kDelta);
        assertEquals(16.333, k.getMaxCapsizeAccelM_S2(), kDelta);
    }

    @Test
    void testDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(4, maxV, kDelta);
        assertEquals(11.313, maxOmega, kDelta);
        {
            // all translation at the limit -> no effect
            ChassisSpeeds s = new ChassisSpeeds(4, 0, 0);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(4, ms[0].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // all translation over the limit -> clip
            ChassisSpeeds s = new ChassisSpeeds(5, 0, 0);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(5, ms[0].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // all rotation at the limit -> no effect
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 11.313);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(4, ms[0].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // all rotation over the limit -> clip
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 12);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(4.243, ms[0].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testDesaturation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        {
            // half speed in both -> no effect
            ChassisSpeeds s = new ChassisSpeeds(2, 0, 5.656);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // not full speed because wheels are at 45 from course
            assertEquals(1.531, ms[0].speedMetersPerSecond, kDelta);
            assertEquals(3.695, ms[1].speedMetersPerSecond, kDelta);
            assertEquals(1.531, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(3.695, ms[3].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(2, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // half speed in both at 45 -> no effect
            ChassisSpeeds s = new ChassisSpeeds(1.414, 1.414, 5.656);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // "front"
            assertEquals(2.828, ms[0].speedMetersPerSecond, kDelta);
            // outside
            assertEquals(4, ms[1].speedMetersPerSecond, kDelta);
            // inside
            assertEquals(0, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(2.828, ms[3].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // full speed in both at 45 should be the same.
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 11.313);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // "front"
            assertEquals(5.656, ms[0].speedMetersPerSecond, kDelta);
            // outside
            assertEquals(8, ms[1].speedMetersPerSecond, kDelta);
            // inside
            assertEquals(0, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(5.656, ms[3].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.657, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testDesaturation3() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        {
            // full translation, half rotation preserves the ratio.
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 5.656);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            // "front"
            assertEquals(4.471, ms[0].speedMetersPerSecond, kDelta);
            // outside
            assertEquals(6, ms[1].speedMetersPerSecond, kDelta);
            // inside
            assertEquals(2, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(4.471, ms[3].speedMetersPerSecond, kDelta);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            // wanted full, 4, got 2.666 which is 2/3 of the ask, 2/3 of max
            assertEquals(1.885, i.vxMetersPerSecond, kDelta);
            assertEquals(1.885, i.vyMetersPerSecond, kDelta);
            // wanted half, 5.656, got 3.771 which is 2/3 of the ask, 1/3 of max
            assertEquals(3.771, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(4, maxV, kDelta);
        assertEquals(11.313, maxOmega, kDelta);
        // same cases as above

        {
            ChassisSpeeds s = new ChassisSpeeds(4, 0, 0);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(5, 0, 0);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(4, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(0, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 11.313);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(0, 0, 12);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(11.313, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        {
            ChassisSpeeds s = new ChassisSpeeds(2, 0, 5.656);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(2, i.vxMetersPerSecond, kDelta);
            assertEquals(0, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(1.414, 1.414, 5.656);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.656, i.omegaRadiansPerSecond, kDelta);
        }
        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 11.313);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(1.414, i.vxMetersPerSecond, kDelta);
            assertEquals(1.414, i.vyMetersPerSecond, kDelta);
            assertEquals(5.657, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAnalyticDesaturation3() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();

        {
            ChassisSpeeds s = new ChassisSpeeds(2.828, 2.828, 5.656);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(1.885, i.vxMetersPerSecond, kDelta);
            assertEquals(1.885, i.vyMetersPerSecond, kDelta);
            assertEquals(3.771, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testAFewCases() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(4, maxV, kDelta);
        assertEquals(11.313, maxOmega, kDelta);

        {
            // with no translation the wheel speed is ok
            ChassisSpeeds s = new ChassisSpeeds(0, 0, -9.38);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(3.316, ms[0].speedMetersPerSecond, kDelta);
            assertEquals(3.316, ms[1].speedMetersPerSecond, kDelta);
            assertEquals(3.316, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(3.316, ms[3].speedMetersPerSecond, kDelta);
            // with an extra ~2m/s, it's too fast
            s = new ChassisSpeeds(0.13, -1.95, -9.38);
            ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            assertEquals(4.957, ms[0].speedMetersPerSecond, kDelta);
            assertEquals(4.832, ms[1].speedMetersPerSecond, kDelta);
            assertEquals(2.506, ms[2].speedMetersPerSecond, kDelta);
            assertEquals(2.250, ms[3].speedMetersPerSecond, kDelta);

            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            ChassisSpeeds i = l.toChassisSpeeds(ms);
            // so it slows down
            assertEquals(0.105, i.vxMetersPerSecond, kDelta);
            assertEquals(-1.574, i.vyMetersPerSecond, kDelta);
            assertEquals(-7.569, i.omegaRadiansPerSecond, kDelta);
        }
        {
            // the other way slows down more because it is pessimistic about theta.
            ChassisSpeeds s = new ChassisSpeeds(0.13, -1.95, -9.38);
            ChassisSpeeds i = l.analyticDesaturation(s);
            assertEquals(0.098, i.vxMetersPerSecond, kDelta);
            assertEquals(-1.480, i.vyMetersPerSecond, kDelta);
            assertEquals(-7.118, i.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testEquivalentDesaturation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(4, maxV, kDelta);
        assertEquals(11.313, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = l.toChassisSpeeds(ms);
            // does not take theta into account
            ChassisSpeeds i2 = l.analyticDesaturation(s);
            // i2 should never be faster
            double x2 = Math.abs(i2.vxMetersPerSecond);
            double x1 = Math.abs(i1.vxMetersPerSecond);
            if (x2 > x1 + 1e-6) {
                Util.printf("X high %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            // but i1 shouldn't be *too* much faster.
            if ((x2 - x1) / x1 > 0.1) {
                Util.printf("X low %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            double y2 = Math.abs(i2.vyMetersPerSecond);
            double y1 = Math.abs(i1.vyMetersPerSecond);
            if (y2 > y1 + 1e-6) {
                Util.printf("Y high %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            if ((y2 - y1) / y1 > 0.1) {
                Util.printf("Y low %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            double o2 = Math.abs(i2.omegaRadiansPerSecond);
            double o1 = Math.abs(i1.omegaRadiansPerSecond);
            if (o2 > o1 + 1e-6) {
                Util.printf("omega high %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
            if ((o2 - o1) / o1 > 0.1) {
                Util.printf("omega low %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
        }
    }

    @Test
    void testEquivalentDesaturationTwist() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        double maxV = l.getMaxDriveVelocityM_S();
        double maxOmega = l.getMaxAngleSpeedRad_S();
        assertEquals(4, maxV, kDelta);
        assertEquals(11.313, maxOmega, kDelta);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            ChassisSpeeds s = new ChassisSpeeds(
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10,
                    random.nextDouble() * 20 - 10);
            SwerveModuleState[] ms = l.toSwerveModuleStatesWithoutDiscretization(s);
            SwerveDriveKinematics100.desaturateWheelSpeeds(ms, maxV);
            // takes theta into account, can go faster sometimes
            ChassisSpeeds i1 = l.toChassisSpeeds(ms);
            // does not take theta into account
            ChassisSpeeds i2 = l.analyticDesaturation(s);
            // i2 should never be faster
            double x2 = Math.abs(i2.vxMetersPerSecond);
            double x1 = Math.abs(i1.vxMetersPerSecond);
            if (x2 > x1 + 1e-6) {
                Util.printf("X high %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            // but i1 shouldn't be *too* much faster.
            if ((x2 - x1) / x1 > 0.1) {
                Util.printf("X low %.8f %.8f\n", x2, x1);
                dump(i, s, i1, i2);
            }
            double y2 = Math.abs(i2.vyMetersPerSecond);
            double y1 = Math.abs(i1.vyMetersPerSecond);
            if (y2 > y1 + 1e-6) {
                Util.printf("Y high %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            if ((y2 - y1) / y1 > 0.1) {
                Util.printf("Y low %.8f %.8f\n", y2, y1);
                dump(i, s, i1, i2);
            }
            double o2 = Math.abs(i2.omegaRadiansPerSecond);
            double o1 = Math.abs(i1.omegaRadiansPerSecond);
            if (o2 > o1 + 1e-6) {
                Util.printf("omega high %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
            if ((o2 - o1) / o1 > 0.1) {
                Util.printf("omega low %.8f %.8f\n", o2, o1);
                dump(i, s, i1, i2);
            }
        }
    }

    @Test
    void testPreferRotation() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        assertEquals(11.313, l.getMaxAngleSpeedRad_S(), kDelta);
        {
            // trivial case works
            FieldRelativeVelocity t = new FieldRelativeVelocity(0, 0, 0);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(0, i.theta(), kDelta);
        }
    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        assertEquals(11.313, l.getMaxAngleSpeedRad_S(), kDelta);
        {
            // inside the envelope => no change
            FieldRelativeVelocity t = new FieldRelativeVelocity(1, 0, 1);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(1, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(1, i.theta(), kDelta);
        }
        {
            // full v, half omega => half v
            FieldRelativeVelocity t = new FieldRelativeVelocity(4, 0, 5.656);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(2, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(5.656, i.theta(), kDelta);
        }
        {
            // full v, full omega => zero v, sorry
            FieldRelativeVelocity t = new FieldRelativeVelocity(4, 0, 11.313);
            FieldRelativeVelocity i = l.preferRotation(t);
            assertEquals(0, i.x(), kDelta);
            assertEquals(0, i.y(), kDelta);
            assertEquals(11.313, i.theta(), kDelta);
        }
    }

    @Test
    void testDiscretizationNoEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // pure rotation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 1);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 1, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, kDelta);
        }
        {
            // pure translation involves no discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 0, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(1, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(0, impliedSpeeds.omegaRadiansPerSecond, kDelta);
        }
    }

    @Test
    void testDiscretizationWithEffect() {
        SwerveKinodynamics l = SwerveKinodynamicsFactory.get();
        // for this test the gyro rate and the commanded omega are the same,
        // though this is definitely not true in general
        {
            // holonomic does have discretization effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 1);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 1, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.999, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.035, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(1, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(1, 0.02, states);
            assertEquals(0.999, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(1, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // more spinning => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 3, 0.02);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.994, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.105, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(3, 0.02, states);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 3, 0.2);
            ChassisSpeeds impliedSpeeds = l.toChassisSpeeds(states);
            assertEquals(0.944, impliedSpeeds.vxMetersPerSecond, kDelta);
            assertEquals(-0.372, impliedSpeeds.vyMetersPerSecond, kDelta);
            assertEquals(3, impliedSpeeds.omegaRadiansPerSecond, kDelta);

            // invert the discretization to extract the original speeds.
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(3, 0.2, states);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
        {
            // longer time interval => bigger effect
            ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 3);
            SwerveModuleState[] states = l.toSwerveModuleStates(speeds, 3, 0.2);
            ChassisSpeeds correctedImplied = l.toChassisSpeedsWithDiscretization(3, 0.2, states);
            assertEquals(1, correctedImplied.vxMetersPerSecond, kDelta);
            assertEquals(0, correctedImplied.vyMetersPerSecond, kDelta);
            assertEquals(3, correctedImplied.omegaRadiansPerSecond, kDelta);
        }
    }

    private void dump(int i, ChassisSpeeds s, ChassisSpeeds i1, ChassisSpeeds i2) {
        Util.printf("%d -- IN: %s OUT1: %s OUT2: %s\n", i, s, i1, i2);
    }

}
