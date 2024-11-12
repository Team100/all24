package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.Model100;
import org.team100.lib.state.State100;

/**
 * Illustrates how to coordinate multiple profiles to take the same amount of
 * time, by slowing down the faster one.
 * 
 * Coordinating multiple profiles with moving end-states involves a bit of
 * complexity as described in Lavalle, https://arxiv.org/pdf/2210.01744.pdf.
 * 
 * The approach here is simpler, and only works for at-rest end-states: in
 * short, you just slow down the max velocity and acceleration of the "faster"
 * profile. :-)
 * 
 * The WPI profile keeps the timing data around after the "calculate" method, so
 * we can look there; our own profile does not, so we need to modify it.
 */
class CoordinatedProfileTest {
    private static final boolean PRINT = false;

    private static final double PROFILE_TOLERANCE = 0.01;
    private static final double kDelta = 0.001;
    private static final double DT = 0.02;

    /**
     * Verify that the profile times are what we think they should be:
     * vel = 1, acc = 1
     * rest-to-rest 1 takes 2 sec with a triangle profile
     * rest-to-rest 2 takes 3 sec with a trapezoid.
     */
    @Test
    void testVerify() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // two profiles with the same parameters
        TrapezoidProfile100 p1 = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        // initial state at the origin at rest
        State100 i1 = new State100(0, 0);
        State100 i2 = new State100(0, 0);
        // final state at 1, at rest
        State100 g1 = new State100(1, 0);
        State100 g2 = new State100(2, 0);

        // how long does it take to get to the first goal?
        State100 s1 = i1;
        double total_time = 0;
        double max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s1 = p1.calculate(DT, s1, g1);
            total_time += DT;
            max_v = Math.max(max_v, s1.v());
            if (s1.near(g1, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s1);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a triangle profile
        assertEquals(2.0, total_time, kDelta);

        // the second goal is farther away.
        State100 s2 = i2;
        total_time = 0;
        max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s2 = p2.calculate(DT, s2, g2);
            total_time += DT;
            max_v = Math.max(max_v, s2.v());
            if (s2.near(g2, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s2);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a trapezoid profile
        assertEquals(3.0, total_time, kDelta);
    }

    /**
     * Same as above for WPI profile.
     */
    @Test
    void testVerifyWPI() {
        final int maxVel = 1;
        final int maxAccel = 1;
        // two profiles with the same parameters
        ProfileWPI p1 = new ProfileWPI(maxVel, maxAccel);
        ProfileWPI p2 = new ProfileWPI(maxVel, maxAccel);
        // initial state at the origin at rest
        State100 i1 = new State100(0, 0);
        State100 i2 = new State100(0, 0);
        // final state at 1, at rest
        State100 g1 = new State100(1, 0);
        State100 g2 = new State100(2, 0);

        // how long does it take to get to the first goal?
        State100 s1 = i1;
        double total_time = 0;
        double max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            // note WPI doesn't produce accel in the profile. :-)
            s1 = p1.calculate(DT, s1, g1);
            total_time += DT;
            max_v = Math.max(max_v, s1.v());
            if (s1.near(g1, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s1);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a triangle profile
        assertEquals(2.0, total_time, kDelta);

        // the second goal is farther away.
        State100 s2 = i2;
        total_time = 0;
        max_v = 0;
        for (int i = 0; i < 1000; ++i) {
            // next state
            s2 = p2.calculate(DT, s2, g2);
            total_time += DT;
            max_v = Math.max(max_v, s2.v());
            if (s2.near(g2, 0.01)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s\n", total_time, s2);
        }
        if (PRINT)
            System.out.println("max v " + max_v);
        // this is a trapezoid profile
        assertEquals(3.0, total_time, kDelta);
    }

    /** Verify that the profile produces the correct ETA for these cases. */
    @Test
    void testETAs() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // two profiles with the same parameters
        TrapezoidProfile100 p1 = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        Model100 s1 = i1;
        ResultWithETA r = p1.calculateWithETA(DT, s1, g1);
        double total_time = r.etaS();
        assertEquals(2.0, total_time, kDelta);

        Model100 s2 = i2;
        r = p2.calculateWithETA(DT, s2, g2);
        total_time = r.etaS();
        assertEquals(3.0, total_time, kDelta);
    }

    /** Same as above for WPI */
    @Test
    void testETAsWPI() {
        final int maxVel = 1;
        final int maxAccel = 1;
        // two profiles with the same parameters
        ProfileWPI p1 = new ProfileWPI(maxVel, maxAccel);
        ProfileWPI p2 = new ProfileWPI(maxVel, maxAccel);
        // initial state at the origin at rest
        Model100 i1 = new Model100(0, 0);
        Model100 i2 = new Model100(0, 0);
        // final state at 1, at rest
        Model100 g1 = new Model100(1, 0);
        Model100 g2 = new Model100(2, 0);

        Model100 s1 = i1;
        ResultWithETA r = p1.calculateWithETA(DT, s1, g1);
        double total_time = r.etaS();
        assertEquals(2.0, total_time, kDelta);

        Model100 s2 = i2;
        r = p2.calculateWithETA(DT, s2, g2);
        total_time = r.etaS();
        assertEquals(3.0, total_time, kDelta);
    }

    @Test
    void testCoordinatedProfiles() {
        final int maxVel = 1;
        final int maxAccel = 1;
        final double tolerance = 0.01;
        // default x and y profiles
        TrapezoidProfile100 px = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        TrapezoidProfile100 py = new TrapezoidProfile100(maxVel, maxAccel, tolerance);
        // initial x state is moving fast
        Model100 ix = new Model100(0, 1);
        // initial y state is stationary
        Model100 iy = new Model100(0, 0);
        // goal x state is still at the origin (i.e. a "slow and back up" profile)
        Model100 gx = new Model100(0, 0);
        // goal y state is not far
        Model100 gy = new Model100(0.5, 0);

        // the "default profiles" produce different ETA's
        ResultWithETA rx = px.calculateWithETA(DT, ix, gx);
        double tx = rx.etaS();
        assertEquals(2.414, tx, kDelta);

        ResultWithETA ry = py.calculateWithETA(DT, iy, gy);
        double ty = ry.etaS();
        assertEquals(1.414, ty, kDelta);

        // the slower ETA is the controlling one

        double slowETA = Math.max(tx, ty);
        assertEquals(2.414, slowETA, kDelta);

        // find the scale parameters for x and y.

        // in the X case, the given ETA is the default ETA
        double sx = TrapezoidProfile100.solveForSlowerETA(
                maxVel, maxAccel, PROFILE_TOLERANCE, DT, ix, gx, slowETA, kDelta);
        // in the Y case, it's slower
        double sy = TrapezoidProfile100.solveForSlowerETA(
                maxVel, maxAccel, PROFILE_TOLERANCE, DT, iy, gy, slowETA, kDelta);

        // this should be 1.0
        assertEquals(1.0, sx, kDelta);
        assertEquals(0.336, sy, kDelta);

        // use the scale parameter to make adjusted profiles
        px = px.scale(sx);
        py = py.scale(sy);

        // then run the profiles to see where they end up

        State100 stateX = ix;
        State100 stateY = iy;
        double total_time = 0;
        for (int i = 0; i < 1000; ++i) {
            total_time += DT;
            stateX = px.calculate(DT, stateX, gx);
            stateY = py.calculate(DT, stateY, gy);
            if (stateX.near(gx, PROFILE_TOLERANCE) && stateY.near(gy, PROFILE_TOLERANCE)) {
                if (PRINT)
                    System.out.println("at goal at t " + total_time);
                break;
            }
            if (PRINT)
                System.out.printf("%f %s %s\n", total_time, stateX, stateY);
        }
    }

}
