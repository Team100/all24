package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

class TrapezoidProfile100Test {
    private static final double kDt = 0.01;
    boolean dump = true;
    private static final double kDelta = 0.001;

    @Test
    void testIntercepts() {
        Constraints c = new Constraints(5, 0.5);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);
        State s = new State(1, 1);
        assertEquals(0, p.c_plus(s), kDelta);
        assertEquals(2, p.c_minus(s), kDelta);

        // more accel
        c = new Constraints(5, 1);
        p = new TrapezoidProfile100(c, 0.01);
        s = new State(1, 1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);

        // negative velocity, result should be the same.
        c = new Constraints(5, 1);
        p = new TrapezoidProfile100(c, 0.01);
        s = new State(1, -1);
        // means less offset
        assertEquals(0.5, p.c_plus(s), kDelta);
        assertEquals(1.5, p.c_minus(s), kDelta);
    }

    // see studies/rrts TestRRTStar7
    @Test
    void testInterceptsFromRRT() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);

        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        assertEquals(0, p.c_minus(new State(0, 0)), 0.001);
        assertEquals(0, p.c_plus(new State(0, 0)), 0.001);

        assertEquals(0.5, p.c_minus(new State(0, 1)), 0.001);
        assertEquals(-0.5, p.c_plus(new State(0, 1)), 0.001);

        assertEquals(1.5, p.c_minus(new State(1, 1)), 0.001);
        assertEquals(0.5, p.c_plus(new State(1, 1)), 0.001);

        assertEquals(-0.5, p.c_minus(new State(-1, 1)), 0.001);
        assertEquals(-1.5, p.c_plus(new State(-1, 1)), 0.001);

        assertEquals(0.5, p.c_minus(new State(0, -1)), 0.001);
        assertEquals(-0.5, p.c_plus(new State(0, -1)), 0.001);

        assertEquals(2, p.c_minus(new State(0, 2)), 0.001);
        assertEquals(-2, p.c_plus(new State(0, 2)), 0.001);

        assertEquals(0.25, p2.c_minus(new State(0, 1)), 0.001);
        assertEquals(-0.25, p2.c_plus(new State(0, 1)), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, p.c_minus(new State(-3, 2)), 0.001);
        assertEquals(0, p.c_plus(new State(2, 2)), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, p.c_minus(new State(-2, 2)), 0.001);
        assertEquals(0, p.c_plus(new State(2, 2)), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, p.c_minus(new State(-1, 2)), 0.001);
        assertEquals(0, p.c_plus(new State(2, 2)), 0.001);
    }

    @Test
    void testQSwitch() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);

        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        assertEquals(0.375, p2.qSwitchIplusGminus(new State(0, 0), new State(0.5, 1.0)), 0.001);
        assertEquals(0.125, p2.qSwitchIminusGplus(new State(0, 0), new State(0.5, 1.0)), 0.001);

        assertEquals(-0.5, p.qSwitchIplusGminus(new State(-3, 2), new State(2, 2)), 0.001);
        assertEquals(0, p.qSwitchIplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new State(-1, 2), new State(2, 2)), 0.001);

        assertEquals(-0.5, p.qSwitchIminusGplus(new State(2, -2), new State(-3, -2)), 0.001);
        assertEquals(0.0, p.qSwitchIminusGplus(new State(2, -2), new State(-2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State(2, -2), new State(-1, -2)), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, p.qSwitchIplusGminus(new State(2, 2), new State(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIplusGminus(new State(-1, 2), new State(2, -2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State(2, 2), new State(-1, 2)), 0.001);
        assertEquals(0.5, p.qSwitchIminusGplus(new State(-1, 2), new State(2, -2)), 0.001);
    }

    /** Verify some switching velocity cases */
    @Test
    void testQDotSwitch2() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State(-1, 2), new State(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State(0, 2), new State(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        // assertEquals(1.414, p2.qDotSwitchIplusGminus(new State(0.5, 2), new
        // State(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(1, 2), new State(-1, 2)), 0.001);
        // assertEquals(0, p2.qDotSwitchIplusGminus(new State(1, 2), new State(-1, 2)),
        // 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-2, 2), new State(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-1, 2), new State(1, 2)), 0.001);
        // assertEquals(0, p2.qDotSwitchIminusGplus(new State(-1, 2), new State(1, 2)),
        // 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        // assertEquals(1.414, p2.qDotSwitchIminusGplus(new State(-0.5, 2), new
        // State(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.0, p2.qDotSwitchIminusGplus(new State(0, 2), new State(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State(1, 2), new State(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);

    }

    @Test
    void testQDotSwitch2a() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State(-1, 2), new State(1, -2)), 0.001);
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State(-0.5, 2), new State(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State(0, 2), new State(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(0.5, 2), new State(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(1, 2), new State(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(2, 2), new State(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-1, 2), new State(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-0.5, 2), new State(0.5, -2)), 0.001);
        // switching at the goal
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new State(0, 2), new State(0, -2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State(0.5, 2), new State(-0.5, -2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State(1, 2), new State(-1, -2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State(2, 2), new State(-2, -2)), 0.001);

    }

    @Test
    void testQDotSwitch2b() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12)
        assertEquals(3.464, p2.qDotSwitchIplusGminus(new State(-2, -2), new State(2, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8)
        assertEquals(2.828, p2.qDotSwitchIplusGminus(new State(-1, -2), new State(1, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6)
        assertEquals(2.449, p2.qDotSwitchIplusGminus(new State(-0.5, -2), new State(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.qDotSwitchIplusGminus(new State(0, -2), new State(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(0.5, -2), new State(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(1, -2), new State(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(2, -2), new State(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-2, -2), new State(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-1, -2), new State(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-0.5, -2), new State(0.5, -2)), 0.001);
        // switches at I
        assertEquals(-2.000, p2.qDotSwitchIminusGplus(new State(0, -2), new State(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State(0.5, -2), new State(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State(1, -2), new State(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State(2, -2), new State(-2, 2)), 0.001);
    }

    @Test
    void testLongT() {
        // if we supply a very long dt, we should end up at the goal
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        Random random = new Random();
        for (int i = 0; i < 10000; ++i) {
            System.out.println("\n================ " + i);
            // random states in the square between (-2,-2) and (2,2)
            State initial = new State(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            State goal = new State(4.0 * random.nextDouble() - 2.0, 4.0 * random.nextDouble() - 2.0);
            State s = p2.calculate(10, initial, goal);
            System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
            // it always gets exactly to the goal
            assertEquals(goal.position, s.position, 0.000001);
            assertEquals(goal.velocity, s.velocity, 0.000001);
        }
    }

    @Test
    void reciprocal() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(-1, 1);
        State goal = new State(-1, -1);
        State s = p2.calculate(10, initial, goal);
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void endEarly() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // in this case, t1 for I+G- is 0, and i think I-G+ is doing the wrong thing.
        // the delta v is 1, accel is 2, so this is a 0.5s solution.
        State initial = new State(-1, 2);
        State goal = new State(-0.25, 1);

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

        State s = p2.calculate(10, initial, goal);
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    // like above but with reciprocal starting point
    @Test
    void endEarly2() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        State initial = new State(-1, -2);
        State goal = new State(-0.25, 1);

        double qdot = p2.qDotSwitchIminusGplus(initial, goal);
        assertEquals(Double.NaN, qdot, 0.001);

        double qdotIpGm = p2.qDotSwitchIplusGminus(initial, goal);
        assertEquals(2, qdotIpGm, 0.001);

        double t1ImGp = p2.t1IminusGplus(initial, goal);
        assertEquals(Double.NaN, t1ImGp, 0.001);

        double t1IpGm = p2.t1IplusGminus(initial, goal);
        assertEquals(2, t1IpGm, 0.001);

        State s = p2.calculate(10, initial, goal);
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void anotherCase() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(1.127310, -0.624930);
        State goal = new State(1.937043, 0.502350);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void yetAnother() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(-1.178601, -1.534504);
        State goal = new State(-0.848954, -1.916583);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void someTcase() {
        // this is an I-G+ path
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(1.655231, 1.967906);
        State goal = new State(0.080954, -1.693829);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void someTcase2() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        State initial = new State(1.747608, -0.147275);
        State goal = new State(1.775148, 0.497717);

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

        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void someTcase3() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(0.985792, 1.340926);
        State goal = new State(-0.350934, -1.949649);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void someTcase4() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(0, 1);
        State goal = new State(0, -1);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    @Test
    void someTcase2a() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        State initial = new State(1.747608, -0.147275);
        State goal = new State(1.775148, 0.497717);
        State s = p2.calculate(10, initial, goal);
        System.out.printf("initial %s goal %s final %s\n", initial, goal, s);
        // it always gets exactly to the goal
        assertEquals(goal.position, s.position, 0.000001);
        assertEquals(goal.velocity, s.velocity, 0.000001);
    }

    /** verify time to velocity limit */
    @Test
    void testVT() {
        // lower max V than the other cases here
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // initial is (-2,2), vmax is 3, u is 2, so time to limit is 0.5.
        // at 0.5, v=2+2*0.5=3. x=-2+2*0.5+0.5*2*(0.5)^2 = -2+1+0.25=-0.75
        // so this is right at the limit, we should just proceed.
        State s = p2.calculate(0.02, new State(-0.75, 3.00), new State(2, 2));
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.6900, s.position, 0.0001);
        // should continue at vmax, not go faster
        assertEquals(3.00, s.velocity, 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new State(0.75, -3.00), new State(-2, -2));
        assertEquals(0.6900, s.position, 0.0001);
        assertEquals(-3.00, s.velocity, 0.001);
    }

    @Test
    void testVT2() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        // if we're *near* the limit then there should be two segments.
        State s = p2.calculate(0.02, new State(-0.78, 2.98), new State(2, 2));
        // follow the profile for about 0.01, then the limit for another 0.01
        // at vmax for 0.02, -0.75+3*0.02 = exactly -0.69, no t^2 term
        assertEquals(-0.7200, s.position, 0.0001);
        // end up at exactly vmax
        assertEquals(3.00, s.velocity, 0.001);

        // same, inverted.
        s = p2.calculate(0.02, new State(0.78, -2.98), new State(-2, -2));
        assertEquals(0.7200, s.position, 0.0001);
        assertEquals(-3.00, s.velocity, 0.001);
    }

    @Test
    void testVT3() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        // if we're at the limit but right at the end, we should join G-.
        State s = p2.calculate(0.02, new State(0.75, 3.00), new State(2, 2));
        // dx = 0.06 - 0.0004
        assertEquals(0.8096, s.position, 0.0001);
        // dv = 0.04
        assertEquals(2.96, s.velocity, 0.001);

        // same, inverted
        s = p2.calculate(0.02, new State(-0.75, -3.00), new State(-2, -2));
        assertEquals(-0.8096, s.position, 0.0001);
        assertEquals(-2.96, s.velocity, 0.001);
    }

    @Test
    void testVT4() {
        Constraints c2 = new Constraints(3, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // if we're *near* the end, there should be two segments.
        // 0.75-0.01*3
        State s = p2.calculate(0.02, new State(0.72, 3.00), new State(2, 2));
        // so for the second 0.01 we should be slowing down
        // x = 0.75 + 0.03 - 0.0001
        // this needs to be exact; we're not taking the tswitch path
        assertEquals(0.7799, s.position, 0.00001);
        // v = 3 - 0.02
        assertEquals(2.98, s.velocity, 0.001);

        // same thing, inverted
        s = p2.calculate(0.02, new State(-0.72, -3.00), new State(-2, -2));
        // for the second segment we should be speeding up
        // x = -0.75 - 0.03 + 0.0001
        assertEquals(-0.7799, s.position, 0.0001);
        // dv = 0.02
        assertEquals(-2.98, s.velocity, 0.001);

    }

    /** Verify the time to the switching point via each path */
    @Test
    void testT() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // dv=1.464, a=2
        assertEquals(0.732, p2.t1IplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        // dv=0.828, a=2
        assertEquals(0.414, p2.t1IplusGminus(new State(-1, 2), new State(1, 2)), 0.001);
        // dv = 0.449, a=2
        assertEquals(0.225, p2.t1IplusGminus(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IplusGminus(new State(0, 2), new State(0, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        // I+G- is negative-time here.
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(1, 2), new State(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(2, 2), new State(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-2, 2), new State(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-1, 2), new State(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        // dv = 0
        assertEquals(0.000, p2.t1IminusGplus(new State(0, 2), new State(0, 2)), 0.001);
        // dv = -4.449, a=2
        assertEquals(2.225, p2.t1IminusGplus(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        // dv = -4.828, a=2
        assertEquals(2.414, p2.t1IminusGplus(new State(1, 2), new State(-1, 2)), 0.001);
        // dv = -5.464
        assertEquals(2.732, p2.t1IminusGplus(new State(2, 2), new State(-2, 2)), 0.001);
    }

    @Test
    void testTa() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        // dv=1.464
        assertEquals(0.732, p2.t1IplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // dv=0.828
        assertEquals(0.414, p2.t1IplusGminus(new State(-1, 2), new State(1, -2)), 0.001);
        assertEquals(0.225, p2.t1IplusGminus(new State(-0.5, 2), new State(0.5, -2)), 0.001);
        // the path switches immediately
        assertEquals(0.000, p2.t1IplusGminus(new State(0, 2), new State(0, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(0.5, 2), new State(-0.5, -2)), 0.001);
        // only the negative-time solution exists
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(1, 2), new State(-1, -2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(2, 2), new State(-2, -2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-1, 2), new State(1, -2)), 0.001);
        // traverses G+ backwards
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-0.5, 2), new State(0.5, -2)), 0.001);
        // switching at the goal, dv=4, a=2
        assertEquals(2.000, p2.t1IminusGplus(new State(0, 2), new State(0, -2)), 0.001);
        // dv=-4.449
        assertEquals(2.225, p2.t1IminusGplus(new State(0.5, 2), new State(-0.5, -2)), 0.001);
        // dv=-4.828
        assertEquals(2.414, p2.t1IminusGplus(new State(1, 2), new State(-1, -2)), 0.001);
        // dv=-5.464
        assertEquals(2.732, p2.t1IminusGplus(new State(2, 2), new State(-2, -2)), 0.001);
    }

    @Test
    void testTb() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // dv=5.464
        assertEquals(2.732, p2.t1IplusGminus(new State(-2, -2), new State(2, 2)), 0.001);
        // dv=4.828
        assertEquals(2.414, p2.t1IplusGminus(new State(-1, -2), new State(1, 2)), 0.001);
        // dv=4.449
        assertEquals(2.225, p2.t1IplusGminus(new State(-0.5, -2), new State(0.5, 2)), 0.001);
        // switches at G
        assertEquals(2.000, p2.t1IplusGminus(new State(0, -2), new State(0, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(0.5, -2), new State(-0.5, 2)), 0.001);
        // traverses G- backwards
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(1, -2), new State(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.t1IplusGminus(new State(2, -2), new State(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-2, -2), new State(2, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-1, -2), new State(1, 2)), 0.001);
        // traverses I- backwards
        assertEquals(Double.NaN, p2.t1IminusGplus(new State(-0.5, -2), new State(0.5, -2)), 0.001);
        // switches at I, dv=0
        assertEquals(0.000, p2.t1IminusGplus(new State(0, -2), new State(0, 2)), 0.001);
        // dv=-0.449
        assertEquals(0.225, p2.t1IminusGplus(new State(0.5, -2), new State(-0.5, 2)), 0.001);
        // dv=-0.828
        assertEquals(0.414, p2.t1IminusGplus(new State(1, -2), new State(-1, 2)), 0.001);
        // dv=-1.464
        assertEquals(0.732, p2.t1IminusGplus(new State(2, -2), new State(-2, 2)), 0.001);
    }

    /** Verify the time to the switching point */
    @Test
    void testT1() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        assertEquals(0.732, p2.t1(new State(-2, 2), new State(2, 2)), 0.001);
        assertEquals(0.414, p2.t1(new State(-1, 2), new State(1, 2)), 0.001);
        assertEquals(0.225, p2.t1(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new State(0, 2), new State(0, 2)), 0.001);
        assertEquals(2.225, p2.t1(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        assertEquals(2.414, p2.t1(new State(1, 2), new State(-1, 2)), 0.001);
        assertEquals(2.732, p2.t1(new State(2, 2), new State(-2, 2)), 0.001);

        assertEquals(0.732, p2.t1(new State(-2, 2), new State(2, -2)), 0.001);
        assertEquals(0.414, p2.t1(new State(-1, 2), new State(1, -2)), 0.001);
        assertEquals(0.225, p2.t1(new State(-0.5, 2), new State(0.5, -2)), 0.001);
        assertEquals(0.000, p2.t1(new State(0, 2), new State(0, -2)), 0.001);
        assertEquals(2.225, p2.t1(new State(0.5, 2), new State(-0.5, -2)), 0.001);
        assertEquals(2.414, p2.t1(new State(1, 2), new State(-1, -2)), 0.001);
        assertEquals(2.732, p2.t1(new State(2, 2), new State(-2, -2)), 0.001);

        assertEquals(2.732, p2.t1(new State(-2, -2), new State(2, 2)), 0.001);
        assertEquals(2.414, p2.t1(new State(-1, -2), new State(1, 2)), 0.001);
        assertEquals(2.225, p2.t1(new State(-0.5, -2), new State(0.5, 2)), 0.001);
        assertEquals(0.000, p2.t1(new State(0, -2), new State(0, 2)), 0.001);
        assertEquals(0.225, p2.t1(new State(0.5, -2), new State(-0.5, 2)), 0.001);
        assertEquals(0.414, p2.t1(new State(1, -2), new State(-1, 2)), 0.001);
        assertEquals(0.732, p2.t1(new State(2, -2), new State(-2, 2)), 0.001);
    }

    /** Verify paths taken */
    @Test
    void testCalculate() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new State(-2, 2), new State(2, 2)).position, 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new State(-1, 2), new State(1, 2)).position, 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new State(-0.5, 2), new State(0.5, 2)).position, 0.001);
        assertEquals(0.000, p2.calculate(0.02, new State(0, 2), new State(0, 2)).position, 0.001);
        assertEquals(0.539, p2.calculate(0.02, new State(0.5, 2), new State(-0.5, 2)).position, 0.001);
        assertEquals(1.039, p2.calculate(0.02, new State(1, 2), new State(-1, 2)).position, 0.001);
        assertEquals(2.039, p2.calculate(0.02, new State(2, 2), new State(-2, 2)).position, 0.001);

        assertEquals(2.04, p2.calculate(0.02, new State(-2, 2), new State(2, 2)).velocity, 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State(-1, 2), new State(1, 2)).velocity, 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State(-0.5, 2), new State(0.5, 2)).velocity, 0.001);
        assertEquals(2.0, p2.calculate(0.02, new State(0, 2), new State(0, 2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(0.5, 2), new State(-0.5, 2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(1, 2), new State(-1, 2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(2, 2), new State(-2, 2)).velocity, 0.001);

    }

    @Test
    void testCalculateA() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        assertEquals(-1.959, p2.calculate(0.02, new State(-2, 2), new State(2, -2)).position, 0.001);
        assertEquals(-0.959, p2.calculate(0.02, new State(-1, 2), new State(1, -2)).position, 0.001);
        assertEquals(-0.459, p2.calculate(0.02, new State(-0.5, 2), new State(0.5, -2)).position, 0.001);
        assertEquals(0.039, p2.calculate(0.02, new State(0, 2), new State(0, -2)).position, 0.001);
        assertEquals(0.539, p2.calculate(0.02, new State(0.5, 2), new State(-0.5, -2)).position, 0.001);
        assertEquals(1.039, p2.calculate(0.02, new State(1, 2), new State(-1, -2)).position, 0.001);
        assertEquals(2.039, p2.calculate(0.02, new State(2, 2), new State(-2, -2)).position, 0.001);

        assertEquals(2.04, p2.calculate(0.02, new State(-2, 2), new State(2, -2)).velocity, 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State(-1, 2), new State(1, -2)).velocity, 0.001);
        assertEquals(2.04, p2.calculate(0.02, new State(-0.5, 2), new State(0.5, -2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(0, 2), new State(0, -2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(0.5, 2), new State(-0.5, -2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(1, 2), new State(-1, -2)).velocity, 0.001);
        assertEquals(1.96, p2.calculate(0.02, new State(2, 2), new State(-2, -2)).velocity, 0.001);
    }

    @Test
    void testCalculateB() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        assertEquals(-2.039, p2.calculate(0.02, new State(-2, -2), new State(2, 2)).position, 0.001);
        assertEquals(-1.039, p2.calculate(0.02, new State(-1, -2), new State(1, 2)).position, 0.001);
        assertEquals(-0.539, p2.calculate(0.02, new State(-0.5, -2), new State(0.5, 2)).position, 0.001);
        assertEquals(-0.039, p2.calculate(0.02, new State(0, -2), new State(0, 2)).position, 0.001);
        assertEquals(0.459, p2.calculate(0.02, new State(0.5, -2), new State(-0.5, 2)).position, 0.001);
        assertEquals(0.959, p2.calculate(0.02, new State(1, -2), new State(-1, 2)).position, 0.001);
        assertEquals(1.959, p2.calculate(0.02, new State(2, -2), new State(-2, 2)).position, 0.001);

        assertEquals(-1.96, p2.calculate(0.02, new State(-2, -2), new State(2, 2)).velocity, 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State(-1, -2), new State(1, 2)).velocity, 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State(-0.5, -2), new State(0.5, 2)).velocity, 0.001);
        assertEquals(-1.96, p2.calculate(0.02, new State(0, -2), new State(0, 2)).velocity, 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State(0.5, -2), new State(-0.5, 2)).velocity, 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State(1, -2), new State(-1, 2)).velocity, 0.001);
        assertEquals(-2.04, p2.calculate(0.02, new State(2, -2), new State(-2, 2)).velocity, 0.001);
    }

    @Test
    void testSwitchingTime() {
        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // between (-2,2) and (2,2) the switching point is at (0, 3.464)
        // at the switching point,
        // u=-2, v=3.464, dt=0.02, dx = 0.0693 + 0.0004, dv=0.04

        // 0.02s before the switching point should yield the switching point exactly
        State s = p2.calculate(0.02, new State(-0.0693, 3.424), new State(2, 2));
        assertEquals(0.000, s.position, 0.001);
        assertEquals(3.464, s.velocity, 0.001);

        // this is right at the switching point: the correct path is 0.02 down G-
        s = p2.calculate(0.02, new State(0, 3.464), new State(2, 2));
        assertEquals(0.0693, s.position, 0.001);
        assertEquals(3.424, s.velocity, 0.001);

        // split dt between I+ and G-
        // u=-2, v=3.464, dt=0.01, dx = 0.0346 + 0.0001, dv=0.02
        // the correct outcome is 0.01 down G-
        s = p2.calculate(0.02, new State(-0.0346, 3.444), new State(2, 2));
        assertEquals(0.0346, s.position, 0.001);
        assertEquals(3.444, s.velocity, 0.001);
    }

    @Test
    void testQDotSwitch() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);

        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);

        assertEquals(1.224, p2.qDotSwitchIplusGminus(new State(0, 0), new State(0.5, 1.0)), 0.001);
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(0, 0), new State(0.5, 1.0)), 0.001);

        assertEquals(3.000, p.qDotSwitchIplusGminus(new State(-3, 2), new State(2, 2)), 0.001);
        assertEquals(2.828, p.qDotSwitchIplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        assertEquals(2.645, p.qDotSwitchIplusGminus(new State(-1, 2), new State(2, 2)), 0.001);

        assertEquals(-3.0, p.qDotSwitchIminusGplus(new State(2, -2), new State(-3, -2)), 0.001);
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new State(2, -2), new State(-2, -2)), 0.001);
        assertEquals(-2.645, p.qDotSwitchIminusGplus(new State(2, -2), new State(-1, -2)), 0.001);

        // from 2,2 to -2,2. There's no intersection between these curves
        assertEquals(Double.NaN, p.qDotSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);
        // assertEquals(0, p.qDotSwitchIplusGminus(new State(2, 2), new State(-2, 2)),
        // 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, p.qDotSwitchIplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(Double.NaN, p.qDotSwitchIminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
        // assertEquals(0, p.qDotSwitchIminusGplus(new State(-2, 2), new State(2, -2)),
        // 0.001);
    }

    @Test
    void testTSwitchByPath() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);

        // from -2 to 2, the 'fast' and normal way
        assertEquals(1.656, p.tSwitchIplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        // this path goes from (-2,2) to (0,0) and then to (2,2)
        assertEquals(Double.NaN, p.tSwitchIminusGplus(new State(-2, 2), new State(2, 2)), 0.001);
        // assertEquals(4, p.tSwitchIminusGplus(new State(-2, 2), new State(2, 2)),
        // 0.001);

        // the opposite order, 2 to -2, this is a completely invalid result,
        // traversing Iplus backwards, and then Gminus backwards.
        assertEquals(Double.NaN, p.tSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);
        // assertEquals(-4, p.tSwitchIplusGminus(new State(2, 2), new State(-2, 2)),
        // 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(9.656, p.tSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);

        // diagonal, the 'fast' and normal way
        assertEquals(5.656, p.tSwitchIplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // this is completely invalid
        assertEquals(Double.NaN, p.tSwitchIminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
        // assertEquals(0, p.tSwitchIminusGplus(new State(-2, 2), new State(2, -2)),
        // 0.001);
        // this is invalid, it should yield like NaN or something
        assertEquals(Double.NaN, p.tSwitchIplusGminus(new State(2, -2), new State(-2, 2)), 0.001);
        // assertEquals(0, p.tSwitchIplusGminus(new State(2, -2), new State(-2, 2)),
        // 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(5.656, p.tSwitchIminusGplus(new State(2, -2), new State(-2, 2)), 0.001);

        // another problem case
        // this can't be done
        assertEquals(Double.NaN, p.tSwitchIminusGplus(new State(-1, 1), new State(0, 0)), 0.001);
        // the "normal" way
        assertEquals(1.449, p.tSwitchIplusGminus(new State(-1, 1), new State(0, 0)), 0.001);

    }

    /**
     * this is a normal profile from 0 to 1, rest-to-rest, it's a triangle profile.
     */
    @Test
    void testTriangle() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.1);
        State sample = new State(0, 0);
        final State end = new State(1, 0);

        double tt = 0;
        // the first sample is near the starting state
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(0, sample.position, kDelta);
        assertEquals(0.04, sample.velocity, kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.position, 0.01);
        assertEquals(1.4, sample.velocity, 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /**
     * this is an inverted profile from 0 to -1, rest-to-rest, it's a triangle
     * profile, it's exactly the inverse of the case above.
     */
    @Test
    void testInvertedTriangle() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);
        State sample = new State(0, 0);
        final State end = new State(-1, 0);

        // the first sample is near the starting state
        Util.printf("%f %f %f\n", 0.0, sample.position, sample.velocity);
        sample = profileX.calculate(0.02, sample, end);
        assertEquals(0, sample.position, kDelta);
        assertEquals(-0.04, sample.velocity, kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
        }
        // halfway there, going fast
        assertEquals(-0.5, sample.position, 0.01);
        assertEquals(-1.4, sample.velocity, kDelta);

        // step to the end of the profile ... this was 0.72.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
        }
        assertEquals(-1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /** with a lower top speed, this profile includes a cruise phase. */
    @Test
    void testCruise() {
        Constraints c = new Constraints(1, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);
        State sample = new State(0, 0);
        final State end = new State(1, 0);

        double tt = 0;

        // the first sample is near the starting state
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0, sample.position, kDelta);
        assertEquals(0.04, sample.velocity, kDelta);

        // step to the cruise phase of the profile
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.25, sample.position, 0.01);
        assertEquals(1.0, sample.velocity, kDelta);

        // step to near the end of cruise
        for (double t = 0; t < 0.5; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.75, sample.position, 0.01);
        assertEquals(1.0, sample.velocity, kDelta);

        // step to the end of the profile // this used to be 0.5
        for (double t = 0; t < 0.66; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /**
     * this is a "u-turn" profile, initially heading away from the goal.
     * overshoot works correctly.
     */
    @Test
    void testUTurn() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);

        // initially heading away from the goal
        State sample = new State(0.1, 1);
        final State end = new State(0, 0);

        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(0.120, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.35, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.19, sample.position, kDelta);
        assertEquals(-0.8, sample.velocity, kDelta);

        // this is the end. this was 0.44
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /** Same as above but not inverted. */
    @Test
    void testUTurnNotInverted() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);

        // initially heading away from the goal
        State sample = new State(-0.1, -1);
        final State end = new State(0, 0);
        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(-0.120, sample.position, kDelta);
        assertEquals(-0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.35, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.19, sample.position, kDelta);
        assertEquals(0.8, sample.velocity, kDelta);

        // this is the end. this was 0.44.
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     * 
     * the WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurn2() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);

        // initially at the goal with nonzero velocity
        State sample = new State(0, 1);
        final State end = new State(0, 0);
        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0.02, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.25, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.1, sample.position, 0.01);
        assertEquals(-0.615, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    /**
     * the same logic should work if the starting position is *at* the goal.
     */
    @Test
    void testUTurn2NotInverted() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);

        // initially at the goal with nonzero velocity
        State sample = new State(0, -1);
        final State end = new State(0, 0);
        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(-0.02, sample.position, kDelta);
        assertEquals(-0.96, sample.velocity, kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.25, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.1, sample.position, 0.01);
        assertEquals(0.615, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.44; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
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
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.01);

        // behind the goal, too fast to stop.
        State sample = new State(-0.1, 1);
        final State end = new State(0, 0);
        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(-0.08, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.15, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.26; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.07, sample.position, 0.01);
        assertEquals(-0.53, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.6; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        // assertTrue(profileX.isFinished());
    }

    @Test
    void testWindupCase() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.05);
        State sample = new State(0, 0);
        final State end = new State(0, 1);
        sample = profileX.calculate(0.02, sample, end);
        // I- means dv = 2 * 0.02 = 0.04 and dx = 0.0004
        assertEquals(-0.0004, sample.position, 0.000001);
        assertEquals(-0.04, sample.velocity, 0.000001);
        sample = profileX.calculate(0.02, sample, end);
        // still I-, dv = 0.04 more, dx = 0.0004 + 0.0008 + 0.0004
        assertEquals(-0.0016, sample.position, 0.000001);
        assertEquals(-0.08, sample.velocity, 0.000001);
    }

    /**
     * initially at rest, we want a state in the same position but moving, so this
     * requires a "windup" u-turn.
     * 
     * The WPI profile fails this test, but the bangbang controller passes.
     */
    @Test
    void testUTurnWindup() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c, 0.05);

        // initially at rest
        State sample = new State(0, 0);
        // goal is moving
        final State end = new State(0, 1);
        double tt = 0;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        // the first sample is near the starting state
        sample = profileX.calculate(0.02, sample, end);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0, sample.position, kDelta);
        assertEquals(-0.04, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.7; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
            if (sample.near(end, 0.05))
                break;

            // if (profileX.isFinished())
            // break;
        }
        assertEquals(-0.25, sample.position, 0.01);
        assertEquals(0.02, sample.velocity, 0.01);

        for (double t = 0; t < 1; t += 0.02) {
            sample = profileX.calculate(0.02, sample, end);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
            if (sample.near(end, 0.05))
                break;
            // if (profileX.isFinished())
            // break;

        }
        assertEquals(0, sample.position, 0.05);
        assertEquals(1, sample.velocity, 0.05);

        // assertTrue(profileX.isFinished());
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
        Constraints constraints = new Constraints(1.75, 0.75);
        final State goal = new State(3, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

    // Tests that decreasing the maximum velocity in the middle when it is already
    // moving faster than the new max is handled correctly
    @Test
    void posContinuousUnderVelChange() {
        Constraints constraints = new Constraints(1.75, 0.75);
        State goal = new State(12, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        State state = profile.calculate(kDt, new State(), goal);

        double lastPos = state.position;
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                constraints = new Constraints(0.75, 0.75);
                profile = new TrapezoidProfile100(constraints, 0.01);
            }

            state = profile.calculate(kDt, state, goal);
            double estimatedVel = (state.position - lastPos) / kDt;

            if (i >= 401) {
                // Since estimatedVel can have floating point rounding errors, we check
                // whether value is less than or within an error delta of the new
                // constraint.
                assertLessThanOrNear(estimatedVel, constraints.maxVelocity, 1e-4);

                assertLessThanOrEquals(state.velocity, constraints.maxVelocity);
            }

            lastPos = state.position;
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

    // There is some somewhat tricky code for dealing with going backwards
    @Test
    void backwards() {
        Constraints constraints = new Constraints(0.75, 0.75);
        final State goal = new State(-2, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

    @Test
    void switchGoalInMiddle() {
        Constraints constraints = new Constraints(0.75, 0.75);
        State goal = new State(-2, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertNotEquals(state, goal);

        goal = new State(0.0, 0.0);
        profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 600; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        Constraints constraints = new Constraints(0.75, 0.75);
        final State goal = new State(4, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertNear(constraints.maxVelocity, state.velocity, 10e-5);

        profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(kDt, state, goal);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

}
