package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.Profile100.ResultWithETA;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.state.State100;
import org.team100.lib.util.Util;

/**
 * Note many of these cases were adjusted slightly to accommodate the treatment
 * of max velocity.
 */
class TrapezoidProfile100Test {
    private static final boolean actuallyPrint = false;
    private static final double k10ms = 0.01;
    private static final double kDelta = 0.001;

    private void dump(double tt, Control100 sample) {
        if (actuallyPrint)
            Util.printf("%f %f %f %f\n", tt, sample.x(), sample.v(), sample.a());
    }

    /** What if the entry velocity is above the cruise velocity? */
    @Test
    void testTooHighEntryVelocity() {
        TrapezoidProfile100 p = new TrapezoidProfile100(1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Model100 initial = new Model100(0, 2);
        // goal is achievable with constant max decel
        Model100 goal = new Model100(2, 0);
        ResultWithETA r = p.calculateWithETA(0.02, initial, goal);
        assertEquals(2, r.etaS(), kDelta);
        // 2m/s * 0.02s = ~0.04
        assertEquals(0.04, r.state().x(), kDelta);
        assertEquals(1.98, r.state().v(), kDelta);
        assertEquals(-1, r.state().a(), kDelta);
        for (double tt = 0.02; tt < 3; tt += 0.02) {
            r = p.calculateWithETA(0.02, r.state().model(), goal);
            dump(tt, r.state());
        }
        // at the goal
        assertEquals(0, r.etaS(), kDelta);
        assertEquals(2, r.state().x(), kDelta);
        assertEquals(0, r.state().v(), kDelta);
        assertEquals(0, r.state().a(), kDelta);
    }

    @Test
    void testTooHighEntryVelocityInReverse() {
        TrapezoidProfile100 p = new TrapezoidProfile100(1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Model100 initial = new Model100(0, -2);
        // goal is achievable with constant max decel
        Model100 goal = new Model100(-2, 0);
        ResultWithETA r = p.calculateWithETA(0.02, initial, goal);
        assertEquals(2, r.etaS(), kDelta);
        // 2m/s * 0.02s = ~0.04
        assertEquals(-0.04, r.state().x(), kDelta);
        assertEquals(-1.98, r.state().v(), kDelta);
        assertEquals(1, r.state().a(), kDelta);
        for (double tt = 0.02; tt < 3; tt += 0.02) {
            r = p.calculateWithETA(0.02, r.state().model(), goal);
            dump(tt, r.state());
        }
        // at the goal
        assertEquals(0, r.etaS(), kDelta);
        assertEquals(-2, r.state().x(), kDelta);
        assertEquals(0, r.state().v(), kDelta);
        assertEquals(0, r.state().a(), kDelta);
    }

    @Test
    void testTooHighEntryVelocityCruising() {
        TrapezoidProfile100 p = new TrapezoidProfile100(1, 1, 0.01);
        // initial state velocity is higher than profile cruise
        Model100 initial = new Model100(0, 2);
        // goal is achievable with max decel 1s, cruise 1s, max decel 1s
        Model100 goal = new Model100(3, 0);
        ResultWithETA r = p.calculateWithETA(0.02, initial, goal);
        assertEquals(3, r.etaS(), kDelta);
        // 2m/s * 0.02s = ~0.04
        assertEquals(0.04, r.state().x(), kDelta);
        assertEquals(1.98, r.state().v(), kDelta);
        assertEquals(-1, r.state().a(), kDelta);
        for (double tt = 0.02; tt < 4; tt += 0.02) {
            r = p.calculateWithETA(0.02, r.state().model(), goal);
            dump(tt, r.state());
        }
        // at the goal
        assertEquals(0, r.etaS(), kDelta);
        assertEquals(3, r.state().x(), kDelta);
        assertEquals(0, r.state().v(), kDelta);
        assertEquals(0, r.state().a(), kDelta);
    }

    /////////////////////////
    //
    // tests about duration
    //

    /** If you're at the goal, the ETA is zero. */
    @Test
    void testETAAtGoal() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(0, 0); // same
        ResultWithETA r = p2.calculateWithETA(0.02, initial, goal);
        // the next state is just the goal
        assertEquals(0, r.state().x(), kDelta);
        // and it takes zero time
        assertEquals(0, r.etaS(), kDelta);
    }

    /** Simple rest-to-rest case */
    @Test
    void testETARestToRest() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(0.0002, s.state().x(), kDelta);
        assertEquals(0.02, s.state().v(), kDelta);
        assertEquals(1, s.state().a(), kDelta);
        // this is a triangular velocity profile
        assertEquals(2, s.etaS(), kDelta);
    }

    /**
     * How can we find parameters that *do* achieve the duration goal?
     * 
     * Scale acceleration. Scaling velocity is not as good, because you could scale
     * it instantly below the initial velocity.
     */
    @Test
    void testETASolve() {
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        // this this is the default eta above, so s = 1.0.
        double s = TrapezoidProfile100.solveForSlowerETA(1, 1, 0.01, 0.02, initial, goal, 2, kDelta);
        assertEquals(1.0, s, kDelta);
        s = TrapezoidProfile100.solveForSlowerETA(1, 1, 0.01, 0.02, initial, goal, 3, kDelta);
        assertEquals(0.439, s, kDelta);
        s = TrapezoidProfile100.solveForSlowerETA(1, 1, 0.01, 0.02, initial, goal, 4, kDelta);
        assertEquals(0.242, s, kDelta);
        s = TrapezoidProfile100.solveForSlowerETA(1, 1, 0.01, 0.02, initial, goal, 8, kDelta);
        assertEquals(0.053, s, kDelta);
    }

    @Test
    void testETASolveStationary() {
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(0, 0);
        // this this is the default eta above, so s = 1.0.
        double s = TrapezoidProfile100.solveForSlowerETA(1, 1, 0.01, 0.02, initial, goal, 2, kDelta);
        assertEquals(1.0, s, kDelta);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled1() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(0.5, 1, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(0.0, s.state().x(), kDelta);
        assertEquals(0.02, s.state().v(), kDelta);
        assertEquals(1, s.state().a(), kDelta);
        assertEquals(2.5, s.etaS(), kDelta);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(0.5, 0.5, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(0.0, s.state().x(), kDelta);
        assertEquals(0.01, s.state().v(), kDelta);
        assertEquals(0.5, s.state().a(), kDelta);
        // this is a trapezoidal velocity profile
        assertEquals(3, s.etaS(), kDelta);
    }

    /** ETA is not a trivial function of V and A */
    @Test
    void testETARestToRestScaled3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(0.25, 0.25, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(0.0, s.state().x(), kDelta);
        assertEquals(0.005, s.state().v(), kDelta);
        assertEquals(0.25, s.state().a(), kDelta);
        // this is a trapezoidal velocity profile
        assertEquals(5, s.etaS(), kDelta);
    }

    /** Initially at max V, cruise and then slow to a stop */
    @Test
    void testETACruise() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, 1); // cruising at maxV
        Model100 goal = new Model100(1, 0); // want to go 1m, so cruise for 0.5m, 0.5s, then slow for 1s
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        // the next state should be a small step in the direction of the goal
        assertEquals(0.02, s.state().x(), kDelta);
        // at the initial velocity
        assertEquals(1, s.state().v(), kDelta);
        // still cruising for now
        assertEquals(0, s.state().a(), kDelta);
        // cruise for 0.5s, then slow for 1s
        assertEquals(1.5, s.etaS(), kDelta);
    }

    /** Initially at max V, slow immediately */
    @Test
    void testETACruiseGMinus() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, 1); // cruising at maxV
        Model100 goal = new Model100(0.5, 0); // want to go 0.5m, so we're on G-
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        // still moving at roughly initial v
        assertEquals(0.02, s.state().x(), kDelta);
        // slowing
        assertEquals(0.98, s.state().v(), kDelta);
        // braking
        assertEquals(-1, s.state().a(), kDelta);
        // will take 1s
        assertEquals(1, s.etaS(), kDelta);
    }

    /** Initially at cruise, goal is the same position */
    @Test
    void testETAReverse() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, 1);
        Model100 goal = new Model100(0, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        // initial velocity carries us forward
        assertEquals(0.02, s.state().x(), kDelta);
        // starting to slow down
        assertEquals(0.98, s.state().v(), kDelta);
        // max braking
        assertEquals(-1, s.state().a(), kDelta);
        // then slow to a stop for 1s (0.5m), then back up and stop (1.4s) so 2.414
        // total
        assertEquals(2.414, s.etaS(), kDelta);
    }

    /** Same as above in the other direction */
    @Test
    void testETACruiseMinus() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, -1);
        Model100 goal = new Model100(-1, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(-0.02, s.state().x(), kDelta);
        assertEquals(-1, s.state().v(), kDelta);
        assertEquals(0, s.state().a(), kDelta);
        assertEquals(1.5, s.etaS(), kDelta);
    }

    /** Same as above in the other direction */
    @Test
    void testETACruiseMinusGPlus() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(1, 1, 0.01);
        Model100 initial = new Model100(0, -1);
        Model100 goal = new Model100(-0.5, 0);
        ResultWithETA s = p2.calculateWithETA(0.02, initial, goal);
        assertEquals(-0.02, s.state().x(), kDelta);
        assertEquals(-0.98, s.state().v(), kDelta);
        assertEquals(1, s.state().a(), kDelta);
        assertEquals(1, s.etaS(), kDelta);
    }

    //////////////////////
    //
    // tests about the new profile
    //

    /** Now we expose acceleration in the profile state, so make sure it's right. */
    @Test
    void testAccel1() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        // 0.5 * 2 * 0.02 * 0.02 = 0.0004
        assertEquals(0.0004, s.x(), 0.000001);
        // 2 * 0.02 = 0.04
        assertEquals(0.04, s.v(), 0.000001);
        // I+ a=2
        assertEquals(2, s.a(), 0.000001);
    }

    @Test
    void testAccel2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // inverted
        Model100 initial = new Model100(0, 0);
        Model100 goal = new Model100(-1, 0);
        Control100 s = p2.calculate(0.02, initial, goal);
        // 0.5 * 2 * 0.02 * 0.02 = 0.0004
        assertEquals(-0.0004, s.x(), 0.000001);
        // 2 * 0.02 = 0.04
        assertEquals(-0.04, s.v(), 0.000001);
        // I+ a=2
        assertEquals(-2, s.a(), 0.000001);
    }

    @Test
    void testAccel3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // cruising
        Model100 initial = new Model100(0, 3);
        Model100 goal = new Model100(10, 0);
        Control100 s = p2.calculate(1, initial, goal);
        // cruising at 3 for 1
        assertEquals(3, s.x(), 0.001);
        // cruising
        assertEquals(3, s.v(), 0.000001);
        // cruising => zero accel
        assertEquals(0, s.a(), 0.000001);
    }

    @Test
    void testIntercepts() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 0.5, 0.01);
        Model100 s = new Model100(1, 1);
        assertEquals(0, p.c_plus(s), kDelta);
        assertEquals(2, p.c_minus(s), kDelta);

        // more accel
        p = new TrapezoidProfile100(5, 1, 0.01);
        s = new Model100(1, 1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);

        // negative velocity, result should be the same.
        p = new TrapezoidProfile100(5, 1, 0.01);
        s = new Model100(1, -1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);
    }

    // see studies/rrts TestRRTStar7
    @Test
    void testInterceptsFromRRT() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(0, p.c_minus(new Model100(0, 0)), 0.001);
        assertEquals(0, p.c_plus(new Model100(0, 0)), 0.001);

        assertEquals(0.5, p.c_minus(new Model100(0, 1)), 0.001);
        assertEquals(-0.5, p.c_plus(new Model100(0, 1)), 0.001);

        assertEquals(1.5, p.c_minus(new Model100(1, 1)), 0.001);
        assertEquals(0.5, p.c_plus(new Model100(1, 1)), 0.001);

        assertEquals(-0.5, p.c_minus(new Model100(-1, 1)), 0.001);
        assertEquals(-1.5, p.c_plus(new Model100(-1, 1)), 0.001);

        assertEquals(0.5, p.c_minus(new Model100(0, -1)), 0.001);
        assertEquals(-0.5, p.c_plus(new Model100(0, -1)), 0.001);

        assertEquals(2, p.c_minus(new Model100(0, 2)), 0.001);
        assertEquals(-2, p.c_plus(new Model100(0, 2)), 0.001);

        assertEquals(0.25, p2.c_minus(new Model100(0, 1)), 0.001);
        assertEquals(-0.25, p2.c_plus(new Model100(0, 1)), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, p.c_minus(new Model100(-3, 2)), 0.001);
        assertEquals(0, p.c_plus(new Model100(2, 2)), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, p.c_minus(new Model100(-2, 2)), 0.001);
        assertEquals(0, p.c_plus(new Model100(2, 2)), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, p.c_minus(new Model100(-1, 2)), 0.001);
        assertEquals(0, p.c_plus(new Model100(2, 2)), 0.001);
    }

    @Test
    void testQSwitch() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(0.375, p2.qSwitchIplusGminus(new Model100(0, 0), new Model100(0.5, 1.0)), 0.001);
        assertEquals(0.125, p2.qSwitchIminusGplus(new Model100(0, 0), new Model100(0.5, 1.0)), 0.001);

        assertEquals(-0.5, p.qSwitchIplusGminus(new Model100(-3, 2), new Model100(2, 2)), 0.001);
        assertEquals(0, p.qSwitchIplusGminus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new Model100(-1, 2), new Model100(2, 2)), 0.001);

        assertEquals(-0.5, p.qSwitchIminusGplus(new Model100(2, -2), new Model100(-3, -2)), 0.001);
        assertEquals(0.0, p.qSwitchIminusGplus(new Model100(2, -2), new Model100(-2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Model100(2, -2), new Model100(-1, -2)), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, p.qSwitchIplusGminus(new Model100(2, 2), new Model100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new Model100(-1, 2), new Model100(2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Model100(2, 2), new Model100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new Model100(-1, 2), new Model100(2, -2)), 0.001);
    }

    /** Verify some switching velocity cases */
    @Test
    void testQDotSwitch2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Model100(-1, 2), new Model100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Model100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Model100(0, 2), new Model100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(1, 2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(2, 2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-1, 2), new Model100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.0, p2.qDotSwitchIminusGplus(new Model100(0, 2), new Model100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Model100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Model100(1, 2), new Model100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Model100(2, 2), new Model100(-2, 2)), 0.001);

    }

    @Test
    void testQDotSwitch2a() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Model100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Model100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Model100(0, 2), new Model100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(1, 2), new Model100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(2, 2), new Model100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-1, 2), new Model100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // switching at the goal
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new Model100(0, 2), new Model100(0, -2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Model100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Model100(1, 2), new Model100(-1, -2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Model100(2, 2), new Model100(-2, -2)), 0.001);

    }

    @Test
    void testQDotSwitch2b() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new Model100(-2, -2), new Model100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new Model100(-1, -2), new Model100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new Model100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new Model100(0, -2), new Model100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(1, -2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new Model100(2, -2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-2, -2), new Model100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-1, -2), new Model100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-0.5, -2), new Model100(0.5, -2)), 0.001);
        // switches at I
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new Model100(0, -2), new Model100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new Model100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new Model100(1, -2), new Model100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new Model100(2, -2), new Model100(-2, 2)), 0.001);
    }

    @Test
    void testOneLongT() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        // goal is far, requires (brief) cruising
        Model100 goal = new Model100(5, 0);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.00001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void testOneLongTReverse() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(0, 0);
        // goal is far, requires (brief) cruising
        Model100 goal = new Model100(-5, 0);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.00001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void testManyLongT() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            // random states in the square between (-2,-2) and (2,2)
            Model100 initial = new Model100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            Model100 goal = new Model100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            Control100 s = p2.calculate(10, initial, goal);
            // it always gets exactly to the goal
            assertEquals(goal.x(), s.x(), 0.00001);
            assertEquals(goal.v(), s.v(), 0.000001);
        }
    }

    @Test
    void reciprocal() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(-1, 1);
        Model100 goal = new Model100(-1, -1);
        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void endEarly() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // in this case, t1 for I+G- is 0, and i think I-G+ is doing the wrong thing.
        // the delta v is 1, accel is 2, so this is a 0.5s solution.
        Model100 initial = new Model100(-1, 2);
        Model100 goal = new Model100(-0.25, 1);

        // in this case the I-G+ path switching point is the reciprocal, which isn't
        // what we want,
        // but it shouldn't matter because the I+G- switching point is I, so we should
        // choose that.
        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(-1, qdot, 0.001);
        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);
        // this is the long way around
        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(1.5, t1ImGp, 0.001);
        // since this is zero we should choose I+G-
        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(0, t1IpGm, 0.001);

        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    // like above but with reciprocal starting point
    @Test
    void endEarly2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        Model100 initial = new Model100(-1, -2);
        Model100 goal = new Model100(-0.25, 1);

        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(Double.NaN, qdot, 0.001);

        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);

        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(Double.NaN, t1ImGp, 0.001);

        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(2, t1IpGm, 0.001);

        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void anotherCase() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(1.127310, -0.624930);
        Model100 goal = new Model100(1.937043, 0.502350);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void yetAnother() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(-1.178601, -1.534504);
        Model100 goal = new Model100(-0.848954, -1.916583);
        Control100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase() {
        // this is an I-G+ path
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(1.655231, 1.967906);
        Model100 goal = new Model100(0.080954, -1.693829);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        Model100 initial = new Model100(1.747608, -0.147275);
        Model100 goal = new Model100(1.775148, 0.497717);

        double cplus = p2.c_plus(initial);
        assertEquals(1.742, cplus, 0.001);
        double cminus = p2.c_minus(goal);
        assertEquals(1.837, cminus, 0.001);

        double qSwitch = p2.qSwitchIplusGminus(initial, goal);
        assertEquals(1.789, qSwitch, 0.001);

        double nono = p2.qSwitchIminusGplus(initial, goal);
        assertEquals(1.733, nono, 0.001);

        // G is +v from I, so should be to the right of I+ to have a solution.
        // I+ intercept is 1.742, goal.v is 0.497717
        // surface x is 1.742 + 0.497717^2/(2*2) = 1.803930
        // goal.p is 1.775148 which is to the left so there is no I+G- solution.
        double qDotSwitch = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(Double.NaN, qDotSwitch, 0.0001);

        // the I-G+ path has a small negative switching velocity
        double qDotSwitchImGp = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(-0.282180, qDotSwitchImGp, 0.0001);

        // there is no I+G- path
        double t1 = p2.t1IplusGminus(initial, goal);
        assertEquals(Double.NaN, t1, 0.000001);
        double t1a = p2.t1IminusGplus(initial, goal);
        assertEquals(0.067452, t1a, 0.000001);

        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(0.985792, 1.340926);
        Model100 goal = new Model100(-0.350934, -1.949649);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase4() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(0, 1);
        Model100 goal = new Model100(0, -1);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2a() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Model100 initial = new Model100(1.747608, -0.147275);
        Model100 goal = new Model100(1.775148, 0.497717);
        Control100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    /** verify time to velocity limit */
    @Test
    void testVT() {
        // lower max V than the other cases here
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // initial is (-2,2), vmax is 3, u is 2, so time to limit is 0.5.
        // at 0.5, v=2+2*0.5=3. x=-2+2*0.5+0.5*2*(0.5)^2 = -2+1+0.25=-0.75
        // so this is right at the limit, we should just proceed.
        Control100 s = p2.calculate(0.02, new Model100(-0.75, 3.00), new Model100(2, 2));
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.6900, s.x(), 0.0001);
        // should continue at vmax, not go faster
        assertEquals(3.00, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new Model100(0.75, -3.00), new Model100(-2, -2));
        assertEquals(0.6900, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        // if we're *near* the limit then there should be two segments.
        Control100 s = p2.calculate(0.02, new Model100(-0.78, 2.98), new Model100(2, 2));
        // follow the profile for about 0.01, then the limit for another 0.01
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.7200, s.x(), 0.0001);
        // end up at exactly vmax
        assertEquals(3.00, s.v(), 0.001);

        // same, inverted.
        s = p2.calculate(0.02, new Model100(0.78, -2.98), new Model100(-2, -2));
        assertEquals(0.7200, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        // if we're at the limit but right at the end, we should join G-.
        Control100 s = p2.calculate(0.02, new Model100(0.75, 3.00), new Model100(2, 2));
        // dx = 0.06 - 0.0004
        assertEquals(0.8096, s.x(), 0.0001);
        // dv = 0.04
        assertEquals(2.96, s.v(), 0.001);

        // same, inverted
        s = p2.calculate(0.02, new Model100(-0.75, -3.00), new Model100(-2, -2));
        assertEquals(-0.8096, s.x(), 0.0001);
        assertEquals(-2.96, s.v(), 0.001);
    }

    @Test
    void testVT4() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // if we're *near* the end, there should be two segments.
        // 0.75-0.01*3
        Control100 s = p2.calculate(0.02, new Model100(0.72, 3.00), new Model100(2, 2));
        // so for the second 0.01 we should be slowing down
        // x = 0.75 + 0.03 - 0.0001
        // this needs to be exact; we're not taking the tswitch path
        assertEquals(0.77993, s.x(), 0.0001);
        // v = 3 - 0.02
        assertEquals(2.98, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new Model100(-0.72, -3.00), new Model100(-2, -2));
        // for the second segment we should be speeding up
        // x = -0.75 - 0.03 + 0.0001
        assertEquals(-0.7799, s.x(), 0.0001);
        // dv = 0.02
        assertEquals(-2.98, s.v(), 0.001);

    }

    /** Verify the time to the switching point via each path */
    @Test
    void testT() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // dv=1.464, a=2
        assertEquals(0.732, p2.t1IplusGminus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        // dv=0.828, a=2
        assertEquals(0.414, p2.t1IplusGminus(new Model100(-1, 2), new Model100(1, 2)), 0.001);
        // dv = 0.449, a=2
        assertEquals(0.225, p2.t1IplusGminus(new Model100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IplusGminus(new Model100(0, 2), new Model100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(1, 2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(2, 2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-1, 2), new Model100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IminusGplus(new Model100(0, 2), new Model100(0, 2)), 0.001);
        // dv = -4.449, a=2
        assertEquals(2.225, p2.t1IminusGplus(new Model100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        // dv = -4.828, a=2
        assertEquals(2.414, p2.t1IminusGplus(new Model100(1, 2), new Model100(-1, 2)), 0.001);
        // dv = -5.464
        assertEquals(2.732, p2.t1IminusGplus(new Model100(2, 2), new Model100(-2, 2)), 0.001);
    }

    @Test
    void testTa() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        // dv=1.464
        assertEquals(0.732, p2.t1IplusGminus(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        // dv=0.828
        assertEquals(0.414, p2.t1IplusGminus(new Model100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1IplusGminus(new Model100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(0.000, p2.t1IplusGminus(new Model100(0, 2), new Model100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(1, 2), new Model100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(2, 2), new Model100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-1, 2), new Model100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        // switching at the goal, dv=4, a=2
        assertEquals(2.000, p2.t1IminusGplus(new Model100(0, 2), new Model100(0, -2)), 0.001);
        // dv=-4.449
        assertEquals(2.225, p2.t1IminusGplus(new Model100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        // dv=-4.828
        assertEquals(2.414, p2.t1IminusGplus(new Model100(1, 2), new Model100(-1, -2)), 0.001);
        // dv=-5.464
        assertEquals(2.732, p2.t1IminusGplus(new Model100(2, 2), new Model100(-2, -2)), 0.001);
    }

    @Test
    void testTb() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // dv=5.464
        assertEquals(2.732, p2.t1IplusGminus(new Model100(-2, -2), new Model100(2, 2)), 0.001);
        // dv=4.828
        assertEquals(2.414, p2.t1IplusGminus(new Model100(-1, -2), new Model100(1, 2)), 0.001);
        // dv=4.449
        assertEquals(2.225, p2.t1IplusGminus(new Model100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.t1IplusGminus(new Model100(0, -2), new Model100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(1, -2), new Model100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new Model100(2, -2), new Model100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-2, -2), new Model100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-1, -2), new Model100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new Model100(-0.5, -2), new Model100(0.5, -2)), 0.001);
        // switches at I, dv=0
        assertEquals(0.000, p2.t1IminusGplus(new Model100(0, -2), new Model100(0, 2)), 0.001);
        // dv=-0.449
        assertEquals(0.225, p2.t1IminusGplus(new Model100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        // dv=-0.828
        assertEquals(0.414, p2.t1IminusGplus(new Model100(1, -2), new Model100(-1, 2)), 0.001);
        // dv=-1.464
        assertEquals(0.732, p2.t1IminusGplus(new Model100(2, -2), new Model100(-2, 2)), 0.001);
    }

    /** Verify the time to the switching point */
    @Test
    void testT1() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(0.732, p2.t1(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(0.414, p2.t1(new Model100(-1, 2), new Model100(1, 2)), 0.001);
        assertEquals(0.225, p2.t1(new Model100(-0.5, 2), new Model100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new Model100(0, 2), new Model100(0, 2)), 0.001);
        assertEquals(2.225, p2.t1(new Model100(0.5, 2), new Model100(-0.5, 2)), 0.001);
        assertEquals(2.414, p2.t1(new Model100(1, 2), new Model100(-1, 2)), 0.001);
        assertEquals(2.732, p2.t1(new Model100(2, 2), new Model100(-2, 2)), 0.001);

        assertEquals(0.732, p2.t1(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        assertEquals(0.414, p2.t1(new Model100(-1, 2), new Model100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1(new Model100(-0.5, 2), new Model100(0.5, -2)), 0.001);
        assertEquals(0.000, p2.t1(new Model100(0, 2), new Model100(0, -2)), 0.001);
        assertEquals(2.225, p2.t1(new Model100(0.5, 2), new Model100(-0.5, -2)), 0.001);
        assertEquals(2.414, p2.t1(new Model100(1, 2), new Model100(-1, -2)), 0.001);
        assertEquals(2.732, p2.t1(new Model100(2, 2), new Model100(-2, -2)), 0.001);

        assertEquals(2.732, p2.t1(new Model100(-2, -2), new Model100(2, 2)), 0.001);
        assertEquals(2.414, p2.t1(new Model100(-1, -2), new Model100(1, 2)), 0.001);
        assertEquals(2.225, p2.t1(new Model100(-0.5, -2), new Model100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new Model100(0, -2), new Model100(0, 2)), 0.001);
        assertEquals(0.225, p2.t1(new Model100(0.5, -2), new Model100(-0.5, 2)), 0.001);
        assertEquals(0.414, p2.t1(new Model100(1, -2), new Model100(-1, 2)), 0.001);
        assertEquals(0.732, p2.t1(new Model100(2, -2), new Model100(-2, 2)), 0.001);
    }

    /** Verify paths taken */
    @Test
    void testCalculate() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new Model100(-2, 2), new Model100(2, 2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new Model100(-1, 2), new Model100(1, 2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new Model100(-0.5, 2), new Model100(0.5, 2)).x(), 0.001);
        assertEquals(0.000, p2.calculate(0.02, new Model100(0, 2), new Model100(0, 2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new Model100(0.5, 2), new Model100(-0.5, 2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new Model100(1, 2), new Model100(-1, 2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new Model100(2, 2), new Model100(-2, 2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new Model100(-2, 2), new Model100(2, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Model100(-1, 2), new Model100(1, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Model100(-0.5, 2), new Model100(0.5, 2)).v(), 0.001);
        assertEquals(2.0, p2.calculate(0.02, new Model100(0, 2), new Model100(0, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(0.5, 2), new Model100(-0.5, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(1, 2), new Model100(-1, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(2, 2), new Model100(-2, 2)).v(), 0.001);

    }

    @Test
    void testCalculateA() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new Model100(-2, 2), new Model100(2, -2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new Model100(-1, 2), new Model100(1, -2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new Model100(-0.5, 2), new Model100(0.5, -2)).x(), 0.001);
        assertEquals(0.039, p2.calculate(0.02, new Model100(0, 2), new Model100(0, -2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new Model100(0.5, 2), new Model100(-0.5, -2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new Model100(1, 2), new Model100(-1, -2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new Model100(2, 2), new Model100(-2, -2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new Model100(-2, 2), new Model100(2, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Model100(-1, 2), new Model100(1, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new Model100(-0.5, 2), new Model100(0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(0, 2), new Model100(0, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(0.5, 2), new Model100(-0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(1, 2), new Model100(-1, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new Model100(2, 2), new Model100(-2, -2)).v(), 0.001);
    }

    @Test
    void testCalculateB() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-2.039, p2.calculate(0.02, new Model100(-2, -2), new Model100(2, 2)).x(), 0.001);
        assertEquals(-1.039, p2.calculate(0.02, new Model100(-1, -2), new Model100(1, 2)).x(), 0.001);
        assertEquals(-0.539, p2.calculate(0.02, new Model100(-0.5, -2), new Model100(0.5, 2)).x(), 0.001);
        assertEquals(-0.039, p2.calculate(0.02, new Model100(0, -2), new Model100(0, 2)).x(), 0.001);
        assertEquals(0.459, p2.calculate(0.02, new Model100(0.5, -2), new Model100(-0.5, 2)).x(), 0.001);
        assertEquals(0.959, p2.calculate(0.02, new Model100(1, -2), new Model100(-1, 2)).x(), 0.001);
        assertEquals(1.959, p2.calculate(0.02, new Model100(2, -2), new Model100(-2, 2)).x(), 0.001);

        assertEquals(-1.96, p2.calculate(0.02, new Model100(-2, -2), new Model100(2, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Model100(-1, -2), new Model100(1, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Model100(-0.5, -2), new Model100(0.5, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new Model100(0, -2), new Model100(0, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Model100(0.5, -2), new Model100(-0.5, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Model100(1, -2), new Model100(-1, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new Model100(2, -2), new Model100(-2, 2)).v(), 0.001);
    }

    @Test
    void testSwitchingTime() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // between (-2,2) and (2,2) the switching point is at (0, 3.464)
        // at the switching point,
        // u=-2, v=3.464, dt=0.02, dx = 0.0693 + 0.0004, dv=0.04

        // 0.02s before the switching point should yield the switching point exactly
        Control100 s = p2.calculate(0.02, new Model100(-0.0693, 3.424), new Model100(2, 2));
        assertEquals(0.000, s.x(), 0.001);
        assertEquals(3.464, s.v(), 0.001);

        // this is right at the switching point: the correct path is 0.02 down G-
        s = p2.calculate(0.02, new Model100(0, 3.464), new Model100(2, 2));
        assertEquals(0.0693, s.x(), 0.001);
        assertEquals(3.424, s.v(), 0.001);

        // split dt between I+ and G-
        // u=-2, v=3.464, dt=0.01, dx = 0.0346 + 0.0001, dv=0.02
        // the correct outcome is 0.01 down G-
        s = p2.calculate(0.02, new Model100(-0.0346, 3.444), new Model100(2, 2));
        assertEquals(0.0346, s.x(), 0.001);
        assertEquals(3.444, s.v(), 0.001);
    }

    @Test
    void testQDotSwitch() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(1.224, p2.qDotSwitchIplusGminus(new Model100(0, 0), new Model100(0.5, 1.0)), 0.001);
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new Model100(0, 0), new Model100(0.5, 1.0)), 0.001);

        assertEquals(3.000, p.qDotSwitchIplusGminus(new Model100(-3, 2), new Model100(2, 2)), 0.001);
        assertEquals(2.828, p.qDotSwitchIplusGminus(new Model100(-2, 2), new Model100(2, 2)), 0.001);
        assertEquals(2.645, p.qDotSwitchIplusGminus(new Model100(-1, 2), new Model100(2, 2)), 0.001);

        assertEquals(-3.0, p.qDotSwitchIminusGplus(new Model100(2, -2), new Model100(-3, -2)), 0.001);
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new Model100(2, -2), new Model100(-2, -2)), 0.001);
        assertEquals(-2.645, p.qDotSwitchIminusGplus(new Model100(2, -2), new Model100(-1, -2)), 0.001);

        // from 2,2 to -2,2. There's no intersection between these curves
        assertEquals(Double.NaN, p.qDotSwitchIplusGminus(new Model100(2, 2), new Model100(-2, 2)), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, p.qDotSwitchIplusGminus(new Model100(-2, 2), new Model100(2, -2)), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new Model100(2, 2), new Model100(-2, 2)), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(Double.NaN, p.qDotSwitchIminusGplus(new Model100(-2, 2), new Model100(2, -2)), 0.001);

    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.1);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        dump(tt, sample);

        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.x(), 0.01);
        assertEquals(1.4, sample.v(), 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * this is an inverted profile from 0 to -1, rest-to-rest, it's a triangle
     * profile, it's exactly the inverse of the case above.
     */
    @Test
    void testInvertedTriangle() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(-1, 0);

        // the first sample is near the starting state
        dump(0, sample);
        sample = profileX.calculate(0.02, sample.model(), end);
        assertEquals(0, sample.x(), kDelta);
        assertEquals(-0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            dump(t, sample);
        }
        // halfway there, going fast
        assertEquals(-0.5, sample.x(), 0.01);
        assertEquals(-1.4, sample.v(), kDelta);

        // step to the end of the profile ... this was 0.72.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            dump(t, sample);
        }
        assertEquals(-1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /** with a lower top speed, this profile includes a cruise phase. */
    @Test
    void testCruise() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(1, 2, 0.01);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(1, 0);

        double tt = 0;

        // the first sample is near the starting state
        dump(tt, sample);

        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the cruise phase of the profile
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.25, sample.x(), 0.01);
        assertEquals(1.0, sample.v(), kDelta);

        // step to near the end of cruise
        for (double t = 0; t < 0.5; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.75, sample.x(), 0.01);
        assertEquals(1.00, sample.v(), kDelta);

        // step to the end of the profile // this used to be 0.5
        for (double t = 0; t < 0.66; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * this is a "u-turn" profile, initially heading away from the goal.
     * overshoot works correctly.
     */
    @Test
    void testUTurn() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);

        // initially heading away from the goal
        Control100 sample = new Control100(0.1, 1);
        final Model100 end = new Model100(0, 0);

        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0.120, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.35, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.19, sample.x(), kDelta);
        assertEquals(-0.8, sample.v(), kDelta);

        // this is the end. this was 0.44
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /** Same as above but not inverted. */
    @Test
    void testUTurnNotInverted() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);

        // initially heading away from the goal
        Control100 sample = new Control100(-0.1, -1, 0);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(-0.120, sample.x(), kDelta);
        assertEquals(-0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.35, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.19, sample.x(), kDelta);
        assertEquals(0.8, sample.v(), kDelta);

        // this is the end. this was 0.44.
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     * 
     * the WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurn2() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);

        // initially at the goal with nonzero velocity
        Control100 sample = new Control100(0, 1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0.02, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.25, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.1, sample.x(), 0.01);
        assertEquals(-0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     */
    @Test
    void testUTurn2NotInverted() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);

        // initially at the goal with nonzero velocity
        Control100 sample = new Control100(0, -1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.02, sample.x(), kDelta);
        assertEquals(-0.96, sample.v(), kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.25, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.1, sample.x(), 0.01);
        assertEquals(0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.44; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /**
     * And it should work if the starting position is to the *left* of the goal, too
     * fast to stop.
     * 
     * The WPI trapezoid profile fails this test, but the bangbang controller
     * passes.
     */
    @Test
    void testUTurn3() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.01);

        // behind the goal, too fast to stop.
        Control100 sample = new Control100(-0.1, 1);
        final Model100 end = new Model100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.08, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.15, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.26; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.07, sample.x(), 0.01);
        assertEquals(-0.53, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.6; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    @Test
    void testWindupCase() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.05);
        Control100 sample = new Control100(0, 0);
        final Model100 end = new Model100(0, 1);
        sample = profileX.calculate(0.02, sample.model(), end);
        // I- means dv = 2 * 0.02 = 0.04 and dx = 0.0004
        assertEquals(-0.0004, sample.x(), 0.000001);
        assertEquals(-0.04, sample.v(), 0.000001);
        sample = profileX.calculate(0.02, sample.model(), end);
        // still I-, dv = 0.04 more, dx = 0.0004 + 0.0008 + 0.0004
        assertEquals(-0.0016, sample.x(), 0.000001);
        assertEquals(-0.08, sample.v(), 0.000001);
    }

    /**
     * initially at rest, we want a state in the same position but moving, so this
     * requires a "windup" u-turn.
     * 
     * The WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurnWindup() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.05);

        // initially at rest
        Control100 sample = new Control100(0, 0);
        // goal is moving
        final Model100 end = new Model100(0, 1);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample.model(), end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0, sample.x(), kDelta);
        assertEquals(-0.04, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.7; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.model().near(end, 0.05))
                break;

            // if (profileX.isFinished())
            // break;
        }
        assertEquals(-0.25, sample.x(), 0.01);
        assertEquals(0.02, sample.v(), 0.01);

        for (double t = 0; t < 1; t += 0.02) {
            sample = profileX.calculate(0.02, sample.model(), end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.model().near(end, 0.05))
                break;
            // if (profileX.isFinished())
            // break;

        }
        assertEquals(0, sample.x(), 0.05);
        assertEquals(1, sample.v(), 0.05);

    }

    //////////////////////////////////////////////////////

    // Tests below are from WPILib TrapezoidProfileTest.

    /**
     * Asserts "val1" is less than or equal to "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     */
    private static void assertLessThanOrEquals(double val1, double val2) {
        assertTrue(val1 <= val2, val1 + " is greater than " + val2);
    }

    private static void assertNear(double val1, double val2, double eps) {
        assertEquals(val1, val2, eps);
    }

    /**
     * Asserts "val1" is less than or within "eps" of "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     * @param eps  Tolerance for whether values are near to each other.
     */
    private static void assertLessThanOrNear(double val1, double val2, double eps) {
        if (val1 <= val2) {
            assertLessThanOrEquals(val1, val2);
        } else {
            assertNear(val1, val2, eps);
        }
    }

    @Test
    void reachesGoal() {
        final Model100 goal = new Model100(3, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(1.75, 0.75, 0.01);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Tests that decreasing the maximum velocity in the middle when it is already
    // moving faster than the new max is handled correctly
    // Oct 20, 2024, this behavior is different now.  It used to
    // immediately clamp the profile to the new maximum, with
    // infinite acceleration, which is a pointless behavior.  Now
    // the new constraint creates max braking.
    @Test
    void posContinuousUnderVelChange() {
        Model100 goal = new Model100(12, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(1.75, 0.75, 0.01);
        Control100 state = profile.calculate(k10ms, new Model100(0, 0), goal);

        double lastPos = state.x();
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                // impose new slower limit
                profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
            }

            state = profile.calculate(k10ms, state.model(), goal);
            double estimatedVel = (state.x() - lastPos) / k10ms;

            // wait 1.35 sec to brake to the new max velocity
            if (i >= 535) {
                // Since estimatedVel can have floating point rounding errors, we check
                // whether value is less than or within an error delta of the new
                // constraint.
                assertLessThanOrNear(estimatedVel, 0.75, 0.01);

                assertLessThanOrEquals(state.v(), 0.75);
            }

            lastPos = state.x();
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // There is some somewhat tricky code for dealing with going backwards
    @Test
    void backwards() {
        final Model100 goal = new Model100(-2, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    @Test
    void switchGoalInMiddle() {
        Model100 goal = new Model100(-2, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertNotEquals(state, goal);

        goal = new Model100(0.0, 0.0);
        profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 600; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        final Model100 goal = new Model100(4, 0);
        Control100 state = new Control100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertNear(0.75, state.v(), 10e-5);

        profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(k10ms, state.model(), goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

}
