package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.State100;
import org.team100.lib.util.Util;

/**
 * Note many of these cases were adjusted slightly to accommodate the treatment
 * of max velocity.
 */
class TrapezoidProfile100Test {
    private static final boolean actuallyPrint = false;
    private static final double kDt = 0.01;
    private static final double kDelta = 0.001;

    private void dump(double tt, State100 sample) {
        if (actuallyPrint)
            Util.printf("%f %f %f\n", tt, sample.x(), sample.v());
    }

    /** Now we expose acceleration in the profile state, so make sure it's right. */
    @Test
    void testAccel() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        {
            State100 initial = new State100(0, 0);
            State100 goal = new State100(1, 0);
            State100 s = p2.calculate(0.02, initial, goal);
            // 0.5 * 2 * 0.02 * 0.02 = 0.0004
            assertEquals(0.0004, s.x(), 0.000001);
            // 2 * 0.02 = 0.04
            assertEquals(0.04, s.v(), 0.000001);
            // I+ a=2
            assertEquals(2, s.a(), 0.000001);
        }
        {
            // inverted
            State100 initial = new State100(0, 0);
            State100 goal = new State100(-1, 0);
            State100 s = p2.calculate(0.02, initial, goal);
            // 0.5 * 2 * 0.02 * 0.02 = 0.0004
            assertEquals(-0.0004, s.x(), 0.000001);
            // 2 * 0.02 = 0.04
            assertEquals(-0.04, s.v(), 0.000001);
            // I+ a=2
            assertEquals(-2, s.a(), 0.000001);
        }
        {
            // cruising
            State100 initial = new State100(0, 3);
            State100 goal = new State100(10, 0);
            State100 s = p2.calculate(1, initial, goal);
            // cruising at 3 for 1
            assertEquals(3.950378, s.x(), 0.000001);
            // cruising
            assertEquals(3, s.v(), 0.000001);
            // cruising => zero accel
            assertEquals(0, s.a(), 0.000001);
        }
    }

    @Test
    void testIntercepts() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 0.5, 0.01);
        State100 s = new State100(1, 1);
        assertEquals(0, p.c_plus(s), kDelta);
        assertEquals(2, p.c_minus(s), kDelta);

        // more accel
        p = new TrapezoidProfile100(5, 1, 0.01);
        s = new State100(1, 1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);

        // negative velocity, result should be the same.
        p = new TrapezoidProfile100(5, 1, 0.01);
        s = new State100(1, -1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);
    }

    // see studies/rrts TestRRTStar7
    @Test
    void testInterceptsFromRRT() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(0, p.c_minus(new State100(0, 0)), 0.001);
        assertEquals(0, p.c_plus(new State100(0, 0)), 0.001);

        assertEquals(0.5, p.c_minus(new State100(0, 1)), 0.001);
        assertEquals(-0.5, p.c_plus(new State100(0, 1)), 0.001);

        assertEquals(1.5, p.c_minus(new State100(1, 1)), 0.001);
        assertEquals(0.5, p.c_plus(new State100(1, 1)), 0.001);

        assertEquals(-0.5, p.c_minus(new State100(-1, 1)), 0.001);
        assertEquals(-1.5, p.c_plus(new State100(-1, 1)), 0.001);

        assertEquals(0.5, p.c_minus(new State100(0, -1)), 0.001);
        assertEquals(-0.5, p.c_plus(new State100(0, -1)), 0.001);

        assertEquals(2, p.c_minus(new State100(0, 2)), 0.001);
        assertEquals(-2, p.c_plus(new State100(0, 2)), 0.001);

        assertEquals(0.25, p2.c_minus(new State100(0, 1)), 0.001);
        assertEquals(-0.25, p2.c_plus(new State100(0, 1)), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, p.c_minus(new State100(-3, 2)), 0.001);
        assertEquals(0, p.c_plus(new State100(2, 2)), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, p.c_minus(new State100(-2, 2)), 0.001);
        assertEquals(0, p.c_plus(new State100(2, 2)), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, p.c_minus(new State100(-1, 2)), 0.001);
        assertEquals(0, p.c_plus(new State100(2, 2)), 0.001);
    }

    @Test
    void testQSwitch() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(0.375, p2.qSwitchIplusGminus(new State100(0, 0), new State100(0.5, 1.0)), 0.001);
        assertEquals(0.125, p2.qSwitchIminusGplus(new State100(0, 0), new State100(0.5, 1.0)), 0.001);

        assertEquals(-0.5, p.qSwitchIplusGminus(new State100(-3, 2), new State100(2, 2)), 0.001);
        assertEquals(0, p.qSwitchIplusGminus(new State100(-2, 2), new State100(2, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new State100(-1, 2), new State100(2, 2)), 0.001);

        assertEquals(-0.5, p.qSwitchIminusGplus(new State100(2, -2), new State100(-3, -2)), 0.001);
        assertEquals(0.0, p.qSwitchIminusGplus(new State100(2, -2), new State100(-2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State100(2, -2), new State100(-1, -2)), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, p.qSwitchIplusGminus(new State100(2, 2), new State100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new State100(-1, 2), new State100(2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State100(2, 2), new State100(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State100(-1, 2), new State100(2, -2)), 0.001);
    }

    /** Verify some switching velocity cases */
    @Test
    void testQDotSwitch2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State100(-2, 2), new State100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State100(-1, 2), new State100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State100(-0.5, 2), new State100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State100(0, 2), new State100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(0.5, 2), new State100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(1, 2), new State100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(2, 2), new State100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-2, 2), new State100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-1, 2), new State100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-0.5, 2), new State100(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.0, p2.qDotSwitchIminusGplus(new State100(0, 2), new State100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State100(0.5, 2), new State100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State100(1, 2), new State100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State100(2, 2), new State100(-2, 2)), 0.001);

    }

    @Test
    void testQDotSwitch2a() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State100(-2, 2), new State100(2, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State100(-1, 2), new State100(1, -2)), 0.001);
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State100(-0.5, 2), new State100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State100(0, 2), new State100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(0.5, 2), new State100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(1, 2), new State100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(2, 2), new State100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-2, 2), new State100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-1, 2), new State100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-0.5, 2), new State100(0.5, -2)), 0.001);
        // switching at the goal
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new State100(0, 2), new State100(0, -2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State100(0.5, 2), new State100(-0.5, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State100(1, 2), new State100(-1, -2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State100(2, 2), new State100(-2, -2)), 0.001);

    }

    @Test
    void testQDotSwitch2b() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State100(-2, -2), new State100(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State100(-1, -2), new State100(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State100(-0.5, -2), new State100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State100(0, -2), new State100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(0.5, -2), new State100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(1, -2), new State100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State100(2, -2), new State100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-2, -2), new State100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-1, -2), new State100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-0.5, -2), new State100(0.5, -2)), 0.001);
        // switches at I
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new State100(0, -2), new State100(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State100(0.5, -2), new State100(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State100(1, -2), new State100(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State100(2, -2), new State100(-2, 2)), 0.001);
    }

    @Test
    void testLongT() {
        // if we supply a very long dt, we should end up at the goal
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            // random states in the square between (-2,-2) and (2,2)
            State100 initial = new State100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            State100 goal = new State100(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            State100 s = p2.calculate(10, initial, goal);
            // it always gets exactly to the goal
            assertEquals(goal.x(), s.x(), 0.00001);
            assertEquals(goal.v(), s.v(), 0.000001);
        }
    }

    @Test
    void reciprocal() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(-1, 1);
        State100 goal = new State100(-1, -1);
        State100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void endEarly() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // in this case, t1 for I+G- is 0, and i think I-G+ is doing the wrong thing.
        // the delta v is 1, accel is 2, so this is a 0.5s solution.
        State100 initial = new State100(-1, 2);
        State100 goal = new State100(-0.25, 1);

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

        State100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    // like above but with reciprocal starting point
    @Test
    void endEarly2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        State100 initial = new State100(-1, -2);
        State100 goal = new State100(-0.25, 1);

        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(Double.NaN, qdot, 0.001);

        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);

        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(Double.NaN, t1ImGp, 0.001);

        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(2, t1IpGm, 0.001);

        State100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void anotherCase() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(1.127310, -0.624930);
        State100 goal = new State100(1.937043, 0.502350);
        State100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void yetAnother() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(-1.178601, -1.534504);
        State100 goal = new State100(-0.848954, -1.916583);
        State100 s = p2.calculate(10, initial, goal);
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase() {
        // this is an I-G+ path
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(1.655231, 1.967906);
        State100 goal = new State100(0.080954, -1.693829);
        State100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        State100 initial = new State100(1.747608, -0.147275);
        State100 goal = new State100(1.775148, 0.497717);

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

        State100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(0.985792, 1.340926);
        State100 goal = new State100(-0.350934, -1.949649);
        State100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase4() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(0, 1);
        State100 goal = new State100(0, -1);
        State100 s = p2.calculate(10, initial, goal);
        // it always gets exactly to the goal
        assertEquals(goal.x(), s.x(), 0.000001);
        assertEquals(goal.v(), s.v(), 0.000001);
    }

    @Test
    void someTcase2a() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        State100 initial = new State100(1.747608, -0.147275);
        State100 goal = new State100(1.775148, 0.497717);
        State100 s = p2.calculate(10, initial, goal);
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
        State100 s = p2.calculate(0.02, new State100(-0.75, 3.00), new State100(2, 2));
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.6900, s.x(), 0.0001);
        // should continue at vmax, not go faster
        assertEquals(3.00, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new State100(0.75, -3.00), new State100(-2, -2));
        assertEquals(0.6900, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT2() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        // if we're *near* the limit then there should be two segments.
        State100 s = p2.calculate(0.02, new State100(-0.78, 2.98), new State100(2, 2));
        // follow the profile for about 0.01, then the limit for another 0.01
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.7200, s.x(), 0.0001);
        // end up at exactly vmax
        assertEquals(3.00, s.v(), 0.001);

        // same, inverted.
        s = p2.calculate(0.02, new State100(0.78, -2.98), new State100(-2, -2));
        assertEquals(0.7200, s.x(), 0.0001);
        assertEquals(-3.00, s.v(), 0.001);
    }

    @Test
    void testVT3() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);

        // if we're at the limit but right at the end, we should join G-.
        State100 s = p2.calculate(0.02, new State100(0.75, 3.00), new State100(2, 2));
        // dx = 0.06 - 0.0004
        assertEquals(0.8096, s.x(), 0.0001);
        // dv = 0.04
        assertEquals(2.96, s.v(), 0.001);

        // same, inverted
        s = p2.calculate(0.02, new State100(-0.75, -3.00), new State100(-2, -2));
        assertEquals(-0.8096, s.x(), 0.0001);
        assertEquals(-2.96, s.v(), 0.001);
    }

    @Test
    void testVT4() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(3, 2, 0.01);
        // if we're *near* the end, there should be two segments.
        // 0.75-0.01*3
        State100 s = p2.calculate(0.02, new State100(0.72, 3.00), new State100(2, 2));
        // so for the second 0.01 we should be slowing down
        // x = 0.75 + 0.03 - 0.0001
        // this needs to be exact; we're not taking the tswitch path
        assertEquals(0.77993, s.x(), 0.00001);
        // v = 3 - 0.02
        assertEquals(2.98, s.v(), 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new State100(-0.72, -3.00), new State100(-2, -2));
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
        assertEquals(0.732, p2.t1IplusGminus(new State100(-2, 2), new State100(2, 2)), 0.001);
        // dv=0.828, a=2
        assertEquals(0.414, p2.t1IplusGminus(new State100(-1, 2), new State100(1, 2)), 0.001);
        // dv = 0.449, a=2
        assertEquals(0.225, p2.t1IplusGminus(new State100(-0.5, 2), new State100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IplusGminus(new State100(0, 2), new State100(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(0.5, 2), new State100(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(1, 2), new State100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(2, 2), new State100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-2, 2), new State100(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-1, 2), new State100(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-0.5, 2), new State100(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IminusGplus(new State100(0, 2), new State100(0, 2)), 0.001);
        // dv = -4.449, a=2
        assertEquals(2.225, p2.t1IminusGplus(new State100(0.5, 2), new State100(-0.5, 2)), 0.001);
        // dv = -4.828, a=2
        assertEquals(2.414, p2.t1IminusGplus(new State100(1, 2), new State100(-1, 2)), 0.001);
        // dv = -5.464
        assertEquals(2.732, p2.t1IminusGplus(new State100(2, 2), new State100(-2, 2)), 0.001);
    }

    @Test
    void testTa() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        // dv=1.464
        assertEquals(0.732, p2.t1IplusGminus(new State100(-2, 2), new State100(2, -2)), 0.001);
        // dv=0.828
        assertEquals(0.414, p2.t1IplusGminus(new State100(-1, 2), new State100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1IplusGminus(new State100(-0.5, 2), new State100(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(0.000, p2.t1IplusGminus(new State100(0, 2), new State100(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(0.5, 2), new State100(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(1, 2), new State100(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(2, 2), new State100(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-2, 2), new State100(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-1, 2), new State100(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(-0.5, 2), new State100(0.5, -2)), 0.001);
        // switching at the goal, dv=4, a=2
        assertEquals(2.000, p2.t1IminusGplus(new State100(0, 2), new State100(0, -2)), 0.001);
        // dv=-4.449
        assertEquals(2.225, p2.t1IminusGplus(new State100(0.5, 2), new State100(-0.5, -2)), 0.001);
        // dv=-4.828
        assertEquals(2.414, p2.t1IminusGplus(new State100(1, 2), new State100(-1, -2)), 0.001);
        // dv=-5.464
        assertEquals(2.732, p2.t1IminusGplus(new State100(2, 2), new State100(-2, -2)), 0.001);
    }

    @Test
    void testTb() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // dv=5.464
        assertEquals(2.732, p2.t1IplusGminus(new State100(-2, -2), new State100(2, 2)), 0.001);
        // dv=4.828
        assertEquals(2.414, p2.t1IplusGminus(new State100(-1, -2), new State100(1, 2)), 0.001);
        // dv=4.449
        assertEquals(2.225, p2.t1IplusGminus(new State100(-0.5, -2), new State100(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.t1IplusGminus(new State100(0, -2), new State100(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(0.5, -2), new State100(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(1, -2), new State100(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State100(2, -2), new State100(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-2, -2), new State100(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-1, -2), new State100(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State100(-0.5, -2), new State100(0.5, -2)), 0.001);
        // switches at I, dv=0
        assertEquals(0.000, p2.t1IminusGplus(new State100(0, -2), new State100(0, 2)), 0.001);
        // dv=-0.449
        assertEquals(0.225, p2.t1IminusGplus(new State100(0.5, -2), new State100(-0.5, 2)), 0.001);
        // dv=-0.828
        assertEquals(0.414, p2.t1IminusGplus(new State100(1, -2), new State100(-1, 2)), 0.001);
        // dv=-1.464
        assertEquals(0.732, p2.t1IminusGplus(new State100(2, -2), new State100(-2, 2)), 0.001);
    }

    /** Verify the time to the switching point */
    @Test
    void testT1() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(0.732, p2.t1(new State100(-2, 2), new State100(2, 2)), 0.001);
        assertEquals(0.414, p2.t1(new State100(-1, 2), new State100(1, 2)), 0.001);
        assertEquals(0.225, p2.t1(new State100(-0.5, 2), new State100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new State100(0, 2), new State100(0, 2)), 0.001);
        assertEquals(2.225, p2.t1(new State100(0.5, 2), new State100(-0.5, 2)), 0.001);
        assertEquals(2.414, p2.t1(new State100(1, 2), new State100(-1, 2)), 0.001);
        assertEquals(2.732, p2.t1(new State100(2, 2), new State100(-2, 2)), 0.001);

        assertEquals(0.732, p2.t1(new State100(-2, 2), new State100(2, -2)), 0.001);
        assertEquals(0.414, p2.t1(new State100(-1, 2), new State100(1, -2)), 0.001);
        assertEquals(0.225, p2.t1(new State100(-0.5, 2), new State100(0.5, -2)), 0.001);
        assertEquals(0.000, p2.t1(new State100(0, 2), new State100(0, -2)), 0.001);
        assertEquals(2.225, p2.t1(new State100(0.5, 2), new State100(-0.5, -2)), 0.001);
        assertEquals(2.414, p2.t1(new State100(1, 2), new State100(-1, -2)), 0.001);
        assertEquals(2.732, p2.t1(new State100(2, 2), new State100(-2, -2)), 0.001);

        assertEquals(2.732, p2.t1(new State100(-2, -2), new State100(2, 2)), 0.001);
        assertEquals(2.414, p2.t1(new State100(-1, -2), new State100(1, 2)), 0.001);
        assertEquals(2.225, p2.t1(new State100(-0.5, -2), new State100(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new State100(0, -2), new State100(0, 2)), 0.001);
        assertEquals(0.225, p2.t1(new State100(0.5, -2), new State100(-0.5, 2)), 0.001);
        assertEquals(0.414, p2.t1(new State100(1, -2), new State100(-1, 2)), 0.001);
        assertEquals(0.732, p2.t1(new State100(2, -2), new State100(-2, 2)), 0.001);
    }

    /** Verify paths taken */
    @Test
    void testCalculate() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new State100(-2, 2), new State100(2, 2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new State100(-1, 2), new State100(1, 2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new State100(-0.5, 2), new State100(0.5, 2)).x(), 0.001);
        assertEquals(0.000, p2.calculate(0.02, new State100(0, 2), new State100(0, 2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new State100(0.5, 2), new State100(-0.5, 2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new State100(1, 2), new State100(-1, 2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new State100(2, 2), new State100(-2, 2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new State100(-2, 2), new State100(2, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State100(-1, 2), new State100(1, 2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State100(-0.5, 2), new State100(0.5, 2)).v(), 0.001);
        assertEquals(2.0, p2.calculate(0.02, new State100(0, 2), new State100(0, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(0.5, 2), new State100(-0.5, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(1, 2), new State100(-1, 2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(2, 2), new State100(-2, 2)).v(), 0.001);

    }

    @Test
    void testCalculateA() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new State100(-2, 2), new State100(2, -2)).x(), 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new State100(-1, 2), new State100(1, -2)).x(), 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new State100(-0.5, 2), new State100(0.5, -2)).x(), 0.001);
        assertEquals(0.039, p2.calculate(0.02, new State100(0, 2), new State100(0, -2)).x(), 0.001);
        assertEquals(0.539, p2.calculate(0.02, new State100(0.5, 2), new State100(-0.5, -2)).x(), 0.001);
        assertEquals(1.039, p2.calculate(0.02, new State100(1, 2), new State100(-1, -2)).x(), 0.001);
        assertEquals(2.039, p2.calculate(0.02, new State100(2, 2), new State100(-2, -2)).x(), 0.001);

        assertEquals(2.04, p2.calculate(0.02, new State100(-2, 2), new State100(2, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State100(-1, 2), new State100(1, -2)).v(), 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State100(-0.5, 2), new State100(0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(0, 2), new State100(0, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(0.5, 2), new State100(-0.5, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(1, 2), new State100(-1, -2)).v(), 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State100(2, 2), new State100(-2, -2)).v(), 0.001);
    }

    @Test
    void testCalculateB() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        assertEquals(-2.039, p2.calculate(0.02, new State100(-2, -2), new State100(2, 2)).x(), 0.001);
        assertEquals(-1.039, p2.calculate(0.02, new State100(-1, -2), new State100(1, 2)).x(), 0.001);
        assertEquals(-0.539, p2.calculate(0.02, new State100(-0.5, -2), new State100(0.5, 2)).x(), 0.001);
        assertEquals(-0.039, p2.calculate(0.02, new State100(0, -2), new State100(0, 2)).x(), 0.001);
        assertEquals(0.459, p2.calculate(0.02, new State100(0.5, -2), new State100(-0.5, 2)).x(), 0.001);
        assertEquals(0.959, p2.calculate(0.02, new State100(1, -2), new State100(-1, 2)).x(), 0.001);
        assertEquals(1.959, p2.calculate(0.02, new State100(2, -2), new State100(-2, 2)).x(), 0.001);

        assertEquals(-1.96, p2.calculate(0.02, new State100(-2, -2), new State100(2, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State100(-1, -2), new State100(1, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State100(-0.5, -2), new State100(0.5, 2)).v(), 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State100(0, -2), new State100(0, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State100(0.5, -2), new State100(-0.5, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State100(1, -2), new State100(-1, 2)).v(), 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State100(2, -2), new State100(-2, 2)).v(), 0.001);
    }

    @Test
    void testSwitchingTime() {
        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);
        // between (-2,2) and (2,2) the switching point is at (0, 3.464)
        // at the switching point,
        // u=-2, v=3.464, dt=0.02, dx = 0.0693 + 0.0004, dv=0.04

        // 0.02s before the switching point should yield the switching point exactly
        State100 s = p2.calculate(0.02, new State100(-0.0693, 3.424), new State100(2, 2));
        assertEquals(0.000, s.x(), 0.001);
        assertEquals(3.464, s.v(), 0.001);

        // this is right at the switching point: the correct path is 0.02 down G-
        s = p2.calculate(0.02, new State100(0, 3.464), new State100(2, 2));
        assertEquals(0.0693, s.x(), 0.001);
        assertEquals(3.424, s.v(), 0.001);

        // split dt between I+ and G-
        // u=-2, v=3.464, dt=0.01, dx = 0.0346 + 0.0001, dv=0.02
        // the correct outcome is 0.01 down G-
        s = p2.calculate(0.02, new State100(-0.0346, 3.444), new State100(2, 2));
        assertEquals(0.0346, s.x(), 0.001);
        assertEquals(3.444, s.v(), 0.001);
    }

    @Test
    void testQDotSwitch() {
        TrapezoidProfile100 p = new TrapezoidProfile100(5, 1, 0.01);

        TrapezoidProfile100 p2 = new TrapezoidProfile100(5, 2, 0.01);

        assertEquals(1.224, p2.qDotSwitchIplusGminus(new State100(0, 0), new State100(0.5, 1.0)), 0.001);
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State100(0, 0), new State100(0.5, 1.0)), 0.001);

        assertEquals(3.000, p.qDotSwitchIplusGminus(new State100(-3, 2), new State100(2, 2)), 0.001);
        assertEquals(2.828, p.qDotSwitchIplusGminus(new State100(-2, 2), new State100(2, 2)), 0.001);
        assertEquals(2.645, p.qDotSwitchIplusGminus(new State100(-1, 2), new State100(2, 2)), 0.001);

        assertEquals(-3.0, p.qDotSwitchIminusGplus(new State100(2, -2), new State100(-3, -2)), 0.001);
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new State100(2, -2), new State100(-2, -2)), 0.001);
        assertEquals(-2.645, p.qDotSwitchIminusGplus(new State100(2, -2), new State100(-1, -2)), 0.001);

        // from 2,2 to -2,2. There's no intersection between these curves
        assertEquals(Double.NaN, p.qDotSwitchIplusGminus(new State100(2, 2), new State100(-2, 2)), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, p.qDotSwitchIplusGminus(new State100(-2, 2), new State100(2, -2)), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new State100(2, 2), new State100(-2, 2)), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(Double.NaN, p.qDotSwitchIminusGplus(new State100(-2, 2), new State100(2, -2)), 0.001);

    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.1);
        State100 sample = new State100(0, 0);
        final State100 end = new State100(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        dump(tt, sample);

        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.x(), 0.01);
        assertEquals(1.4, sample.v(), 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(0, 0);
        final State100 end = new State100(-1, 0);

        // the first sample is near the starting state
        dump(0, sample);
        sample = profileX.calculate(0.02, sample, end);
        assertEquals(0, sample.x(), kDelta);
        assertEquals(-0.04, sample.v(), kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            dump(t, sample);
        }
        // halfway there, going fast
        assertEquals(-0.5, sample.x(), 0.01);
        assertEquals(-1.4, sample.v(), kDelta);

        // step to the end of the profile ... this was 0.72.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            dump(t, sample);
        }
        assertEquals(-1.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    /** with a lower top speed, this profile includes a cruise phase. */
    @Test
    void testCruise() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(1, 2, 0.01);
        State100 sample = new State100(0, 0);
        final State100 end = new State100(1, 0);

        double tt = 0;

        // the first sample is near the starting state
        dump(tt, sample);

        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0, sample.x(), kDelta);
        assertEquals(0.04, sample.v(), kDelta);

        // step to the cruise phase of the profile
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.25, sample.x(), 0.01);
        assertEquals(1.0, sample.v(), kDelta);

        // step to near the end of cruise
        for (double t = 0; t < 0.5; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.75, sample.x(), 0.01);
        assertEquals(0.999, sample.v(), kDelta);

        // step to the end of the profile // this used to be 0.5
        for (double t = 0; t < 0.66; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(0.1, 1);
        final State100 end = new State100(0, 0);

        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(0.120, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.35, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.19, sample.x(), kDelta);
        assertEquals(-0.8, sample.v(), kDelta);

        // this is the end. this was 0.44
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(-0.1, -1);
        final State100 end = new State100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(-0.120, sample.x(), kDelta);
        assertEquals(-0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.35, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.19, sample.x(), kDelta);
        assertEquals(0.8, sample.v(), kDelta);

        // this is the end. this was 0.44.
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(0, 1);
        final State100 end = new State100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0.02, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.25, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);

        }
        assertEquals(0.1, sample.x(), 0.01);
        assertEquals(-0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(0, -1);
        final State100 end = new State100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.02, sample.x(), kDelta);
        assertEquals(-0.96, sample.v(), kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.25, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(-0.1, sample.x(), 0.01);
        assertEquals(0.615, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.44; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(-0.1, 1);
        final State100 end = new State100(0, 0);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);
        assertEquals(-0.08, sample.x(), kDelta);
        assertEquals(0.96, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.15, sample.x(), kDelta);
        assertEquals(0.0, sample.v(), kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.26; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.07, sample.x(), 0.01);
        assertEquals(-0.53, sample.v(), 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.6; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
        }
        assertEquals(0.0, sample.x(), 0.01);
        assertEquals(0.0, sample.v(), 0.05);
    }

    @Test
    void testWindupCase() {
        TrapezoidProfile100 profileX = new TrapezoidProfile100(5, 2, 0.05);
        State100 sample = new State100(0, 0);
        final State100 end = new State100(0, 1);
        sample = profileX.calculate(0.02, sample, end);
        // I- means dv = 2 * 0.02 = 0.04 and dx = 0.0004
        assertEquals(-0.0004, sample.x(), 0.000001);
        assertEquals(-0.04, sample.v(), 0.000001);
        sample = profileX.calculate(0.02, sample, end);
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
        State100 sample = new State100(0, 0);
        // goal is moving
        final State100 end = new State100(0, 1);
        double tt = 0;
        dump(tt, sample);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        dump(tt, sample);

        assertEquals(0, sample.x(), kDelta);
        assertEquals(-0.04, sample.v(), kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.7; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.near(end, 0.05))
                break;

            // if (profileX.isFinished())
            // break;
        }
        assertEquals(-0.25, sample.x(), 0.01);
        assertEquals(0.02, sample.v(), 0.01);

        for (double t = 0; t < 1; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            dump(tt, sample);
            if (sample.near(end, 0.05))
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
        final State100 goal = new State100(3, 0);
        State100 state = new State100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(1.75, 0.75, 0.01);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Tests that decreasing the maximum velocity in the middle when it is already
    // moving faster than the new max is handled correctly
    @Test
    void posContinuousUnderVelChange() {
        State100 goal = new State100(12, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(1.75, 0.75, 0.01);
        State100 state = profile.calculate(kDt, new State100(0, 0), goal);

        double lastPos = state.x();
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
            }

            state = profile.calculate(kDt, state, goal);
            double estimatedVel = (state.x() - lastPos) / kDt;

            if (i >= 401) {
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
        final State100 goal = new State100(-2, 0);
        State100 state = new State100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    @Test
    void switchGoalInMiddle() {
        State100 goal = new State100(-2, 0);
        State100 state = new State100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertNotEquals(state, goal);

        goal = new State100(0.0, 0.0);
        profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 600; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        final State100 goal = new State100(4, 0);
        State100 state = new State100(0, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertNear(0.75, state.v(), 10e-5);

        profile = new TrapezoidProfile100(0.75, 0.75, 0.01);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.x(), state.x(), 0.05);
        assertEquals(goal.v(), state.v(), 0.05);
    }

}
