package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class SwerveKinodynamicsTest {
    private static final double kDelta = 0.001;

    @Test
    void testComputedValues() {
        double track = 0.5;
        double wheelbase = 0.5;
        double driveV = 1;
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 20 * Math.PI, track, wheelbase, 1);
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
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 20 * Math.PI, track, wheelbase, 1);
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
        SwerveKinodynamics k = new SwerveKinodynamics(driveV, 1, 1, 1, 20 * Math.PI, track, wheelbase, 1);
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
        SwerveKinodynamics k = new SwerveKinodynamics(1, driveA, 1, 1, 20 * Math.PI, track, wheelbase, 1);
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
        SwerveKinodynamics k = new SwerveKinodynamics(1, driveA, 1, 1, 20 * Math.PI, track, wheelbase, 1);
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
        SwerveKinodynamics k = new SwerveKinodynamics(1, 1, 1, 1, 20 * Math.PI, track, wheelbase, vcg);
        assertEquals(1, k.getMaxDriveAccelerationM_S2(), kDelta);

        double fulcrum = Math.min(track / 2, wheelbase / 2);
        assertEquals(0.5, fulcrum, kDelta);

        double accel = 9.8 * fulcrum / vcg;

        assertEquals(16.333, accel, kDelta);
        assertEquals(16.333, k.getMaxCapsizeAccelM_S2(), kDelta);
    }
}
