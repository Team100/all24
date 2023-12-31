package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
        // I+G- is negative-time here. 
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(1, 2), new State(-1, 2)), 0.001);
        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);

        // no intersection
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-2, 2), new State(2, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-1, 2), new State(1, 2)), 0.001);
        // I-G+ is negative-time here
        assertEquals(Double.NaN, p2.qDotSwitchIminusGplus(new State(-0.5, 2), new State(0.5, 2)), 0.001);
        // the same point
        assertEquals(2.0, p2.qDotSwitchIminusGplus(new State(0, 2), new State(0, 2)), 0.001);
        // c(I)=-1.5, x=v^2/4, x=1.5, v=sqrt(6), negative arm
        assertEquals(-2.449, p2.qDotSwitchIminusGplus(new State(0.5, 2), new State(-0.5, 2)), 0.001);
        // c(I)=-2, x=v^2/4, x=2, v=sqrt(8), negative arm
        assertEquals(-2.828, p2.qDotSwitchIminusGplus(new State(1, 2), new State(-1, 2)), 0.001);
        // good path, c(I)=-3, x=v^2/4, x=3, v=sqrt(12) but the negative arm
        assertEquals(-3.464, p2.qDotSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);

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
        assertEquals(0, p.qDotSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, p.qDotSwitchIplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, p.qDotSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(0, p.qDotSwitchIminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
    }

    @Test
    void testTSwitchByPath() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);

        Constraints c2 = new Constraints(5, 2);
        TrapezoidProfile100 p2 = new TrapezoidProfile100(c2, 0.01);
        // from -2 to 2, the 'fast' and normal way
        assertEquals(1.656, p.tSwitchIplusGminus(new State(-2, 2), new State(2, 2)), 0.001);
        // this path goes from (-2,2) to (0,0) and then to (2,2)
        assertEquals(4.000, p.tSwitchIminusGplus(new State(-2, 2), new State(2, 2)), 0.001);

        // the opposite order, 2 to -2, this is a completely invalid result,
        // traversing Iplus backwards, and then Gminus backwards.
        assertEquals(Double.NaN, p.tSwitchIplusGminus(new State(2, 2), new State(-2, 2)), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(9.656, p.tSwitchIminusGplus(new State(2, 2), new State(-2, 2)), 0.001);

        // diagonal, the 'fast' and normal way
        assertEquals(5.656, p.tSwitchIplusGminus(new State(-2, 2), new State(2, -2)), 0.001);
        // this is completely invalid
        assertEquals(Double.NaN, p.tSwitchIminusGplus(new State(-2, 2), new State(2, -2)), 0.001);
        // this is invalid, it should yield like NaN or something
        assertEquals(Double.NaN, p.tSwitchIplusGminus(new State(2, -2), new State(-2, 2)), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(5.656, p.tSwitchIminusGplus(new State(2, -2), new State(-2, 2)), 0.001);

        // another problem case
        // this can't be done
        assertEquals(Double.NaN, p.tSwitchIminusGplus(new State(-1, 1), new State(0, 0)), 0.001);
        // the "normal" way
        assertEquals(1.449, p.tSwitchIplusGminus(new State(-1, 1), new State(0, 0)), 0.001);

    }

    ///////// old tests below

    @Test
    void testBangBangIntercepts() {
        Constraints c = new Constraints(5, 0.5);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);
        State s = new State(1, 1);
        assertEquals(0, p.xplus(s, 0.5), kDelta);
        assertEquals(2, p.xminus(s, 0.5), kDelta);

        // more accel
        c = new Constraints(5, 1);
        p = new TrapezoidProfile100(c, 0.01);
        s = new State(1, 1);
        // means less offset
        assertEquals(0.5, p.xplus(s, 1), kDelta);
        assertEquals(1.5, p.xminus(s, 1), kDelta);

        // negative velocity, result should be the same.
        c = new Constraints(5, 1);
        p = new TrapezoidProfile100(c, 0.01);
        s = new State(1, -1);
        // means less offset
        assertEquals(0.5, p.xplus(s, 1), kDelta);
        assertEquals(1.5, p.xminus(s, 1), kDelta);
    }

    @Test
    void testBangBangU() {
        Constraints c = new Constraints(1, 1);
        TrapezoidProfile100 p = new TrapezoidProfile100(c, 0.01);
        {
            // at the goal, don't do anything
            State goal = new State(1, 1);
            State current = new State(1, 1);
            assertEquals(0, p.u(0.02, goal, current), kDelta);
        }
        {
            // exactly on the switching surface, so slow down
            State goal = new State(1, 0);
            State current = new State(0.5, 1);
            assertEquals(-1, p.u(0.02, goal, current), kDelta);
        }
        {
            // close to the boundary, adjust u
            State goal = new State(1, 0);
            State current = new State(0.5, 0.99);
            assertEquals(-0.497, p.u(0.02, goal, current), kDelta);
        }
        {
            // beyond the goal, so back up
            State goal = new State(1, 1);
            State current = new State(2, 1);
            assertEquals(-1, p.u(0.02, goal, current), kDelta);
        }
        {
            // behind the goal, go forward
            State goal = new State(1, 1);
            State current = new State(0, 1);
            assertEquals(1, p.u(0.02, goal, current), kDelta);

        }
        {
            // at rest, u-turn
            State goal = new State(1, 1);
            State current = new State(1, 0);
            assertEquals(-1, p.u(0.02, goal, current), kDelta);
        }
        {
            // going the other way, apply forward
            State goal = new State(1, 1);
            State current = new State(1, -1);
            assertEquals(1, p.u(0.02, goal, current), kDelta);
        }
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

        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(0, sample.position, kDelta);
        assertEquals(0.04, sample.velocity, kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        // halfway there, going fast
        assertEquals(0.5, sample.position, 0.01);
        assertEquals(1.43, sample.velocity, 0.01);

        // step to the end of the profile .. this was 0.72 before.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        assertEquals(0, sample.position, kDelta);
        assertEquals(-0.04, sample.velocity, kDelta);

        // step to the middle of the profile
        for (double t = 0; t < 0.68; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
        }
        // halfway there, going fast
        assertEquals(-0.5, sample.position, 0.01);
        assertEquals(-1.43, sample.velocity, kDelta);

        // step to the end of the profile ... this was 0.72.
        for (double t = 0; t < 0.86; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
        }
        assertEquals(-1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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

        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0, sample.position, kDelta);
        assertEquals(0.04, sample.velocity, kDelta);

        // step to the cruise phase of the profile
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.25, sample.position, 0.01);
        assertEquals(1.0, sample.velocity, kDelta);

        // step to near the end of cruise
        for (double t = 0; t < 0.5; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.75, sample.position, 0.01);
        assertEquals(1.0, sample.velocity, kDelta);

        // step to the end of the profile // this used to be 0.5
        for (double t = 0; t < 0.66; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(1.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(0.120, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.35, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.19, sample.position, kDelta);
        assertEquals(-0.8, sample.velocity, kDelta);

        // this is the end. this was 0.44
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(-0.120, sample.position, kDelta);
        assertEquals(-0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.35, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.19, sample.position, kDelta);
        assertEquals(0.8, sample.velocity, kDelta);

        // this is the end. this was 0.44.
        for (double t = 0; t < 0.46; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0.02, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.25, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.1, sample.position, 0.01);
        assertEquals(-0.615, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(-0.02, sample.position, kDelta);
        assertEquals(-0.96, sample.velocity, kDelta);

        // step to the turn-around point
        // this takes the same time no matter the starting point.
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.25, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.4; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(-0.1, sample.position, 0.01);
        assertEquals(0.615, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.44; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        assertEquals(-0.08, sample.position, kDelta);
        assertEquals(0.96, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.48; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.15, sample.position, kDelta);
        assertEquals(0.0, sample.velocity, kDelta);

        // the next phase is triangular, this is the point at maximum speed
        // compared to the case above, this is a little sooner and a little slower.
        for (double t = 0; t < 0.26; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.07, sample.position, 0.01);
        assertEquals(-0.53, sample.velocity, 0.01);

        // this is the end.
        // also sooner than the profile above
        for (double t = 0; t < 0.6; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
        }
        assertEquals(0.0, sample.position, 0.01);
        assertEquals(0.0, sample.velocity, 0.05);
        assertTrue(profileX.isFinished());
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
        sample = profileX.calculate(0.02, end, sample);
        tt += 0.02;
        Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);

        assertEquals(0, sample.position, kDelta);
        assertEquals(-0.04, sample.velocity, kDelta);

        // step to the turn-around point
        for (double t = 0; t < 0.7; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
            if (profileX.isFinished())
                break;
        }
        assertEquals(-0.25, sample.position, 0.01);
        assertEquals(0.02, sample.velocity, 0.01);

        for (double t = 0; t < 1; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);
            tt += 0.02;
            Util.printf("%f %f %f\n", tt, sample.position, sample.velocity);
            if (profileX.isFinished())
                break;

        }
        assertEquals(0, sample.position, 0.05);
        assertEquals(1, sample.velocity, 0.05);

        assertTrue(profileX.isFinished());
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
        State goal = new State(3, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(kDt, goal, state);
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
        State state = profile.calculate(kDt, goal, new State());

        double lastPos = state.position;
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                constraints = new Constraints(0.75, 0.75);
                profile = new TrapezoidProfile100(constraints, 0.01);
            }

            state = profile.calculate(kDt, goal, state);
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
        State goal = new State(-2, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(kDt, goal, state);
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
            state = profile.calculate(kDt, goal, state);
        }
        assertNotEquals(state, goal);

        goal = new State(0.0, 0.0);
        profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 600; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        Constraints constraints = new Constraints(0.75, 0.75);
        State goal = new State(4, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertNear(constraints.maxVelocity, state.velocity, 10e-5);

        profile = new TrapezoidProfile100(constraints, 0.01);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(goal.position, state.position, 0.05);
        assertEquals(goal.velocity, state.velocity, 0.05);
    }

}
