package org.team100.lib.rrt;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.awt.Color;
import java.util.List;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;
import org.team100.lib.geom.Obstacle;
import org.team100.lib.geom.Polygon;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNearNode;
import org.team100.lib.index.KDNode;
import org.team100.lib.index.KDTree;
import org.team100.lib.math.Util;
import org.team100.lib.rrt.RRTStar7.Trajectory;
import org.team100.lib.rrt.RRTStar7.Trajectory.Axis;
import org.team100.lib.rrt.example.full_state_arena.FullStateHolonomicArena;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public class TestRRTStar7 {

    @Test
    void testSlowU() {
        System.out.println("faster = higher U, I+G-");
        Axis slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.5);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(4.828, slowU.s1.u, 0.001);
        assertEquals(0.353, slowU.s1.t, 0.001);
        assertEquals(-4.828, slowU.s2.u, 0.001);
        assertEquals(0.146, slowU.s2.t, 0.001);

        System.out.println("example from below, tswitch is 0.724s for U=2, I+G-");
        slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.724);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(2.004, slowU.s1.u, 0.001);
        assertEquals(0.611, slowU.s1.t, 0.001);
        assertEquals(-2.004, slowU.s2.u, 0.001);
        assertEquals(0.112, slowU.s2.t, 0.001);

        System.out.println("just faster than no switch, I+G-");
        slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 0.9);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(1.241, slowU.s1.u, 0.001);
        assertEquals(0.852, slowU.s1.t, 0.001);
        assertEquals(-1.241, slowU.s2.u, 0.001);
        assertEquals(0.047, slowU.s2.t, 0.001); // very short

        System.out.println("no switch I+G-");
        slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 1);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(1.000, slowU.s1.u, 0.001);
        assertEquals(1.000, slowU.s1.t, 0.001);
        assertEquals(-1.000, slowU.s2.u, 0.001);
        assertEquals(0, slowU.s2.t, 0.001); // zero!

        System.out.println("just slower than no switch I-G+");
        slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 1.1);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-0.995, slowU.s1.u, 0.001);
        assertEquals(0.047, slowU.s1.t, 0.001);
        assertEquals(0.995, slowU.s2.u, 0.001);
        assertEquals(1.052, slowU.s2.t, 0.001);

        System.out.println("much slower I-G+");
        slowU = RRTStar7.slowU(0, 0, 0.5, 1.0, 2);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-0.809, slowU.s1.u, 0.001);
        assertEquals(0.381, slowU.s1.t, 0.001);
        assertEquals(0.809, slowU.s2.u, 0.001);
        assertEquals(1.618, slowU.s2.t, 0.001);

        System.out.println("reflect the above case, I+G-");
        slowU = RRTStar7.slowU(0, 0, -0.5, -1.0, 2);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(-0.5, slowU.g, 0.001);
        assertEquals(-1, slowU.gdot, 0.001);
        assertEquals(0.809, slowU.s1.u, 0.001);
        assertEquals(0.381, slowU.s1.t, 0.001);
        assertEquals(-0.809, slowU.s2.u, 0.001);
        assertEquals(1.618, slowU.s2.t, 0.001);

        // single intersection at u=2
        assertEquals(0.414, RRTStar7.tSwitch(0, 1, 0.5, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tLimit(0, 1, 0.5, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tMirror(0, 1, 0.5, 1, 2), 0.001);
        // finds the tswitch u=2 solution, I+G-
        slowU = RRTStar7.slowU(0, 1, 0.5, 1, 0.414);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(2.007, slowU.s1.u, 0.001);
        assertEquals(0.207, slowU.s1.t, 0.001);
        assertEquals(-2.007, slowU.s2.u, 0.001);
        assertEquals(0.207, slowU.s2.t, 0.001); // midpoint
        // finds the tlimit u=2 solution, I-G+
        slowU = RRTStar7.slowU(0, 1, 0.5, 1, 1.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-2.0, slowU.s1.u, 0.001);
        assertEquals(0.5, slowU.s1.t, 0.001);
        assertEquals(2.0, slowU.s2.u, 0.001);
        assertEquals(0.5, slowU.s2.t, 0.001); // midpoint
        // this is a loop below the axis, I-G+
        slowU = RRTStar7.slowU(0, 1, 0.5, 1, 2.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(0.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-1.5, slowU.s1.u, 0.001);
        assertEquals(1.0, slowU.s1.t, 0.001);
        assertEquals(1.5, slowU.s2.u, 0.001);
        assertEquals(1.0, slowU.s2.t, 0.001); // midpoint

        // no limit or mirror at u=2
        assertEquals(0.732, RRTStar7.tSwitch(0, 1, 1, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1, 1, 2), 0.001);
        // discover the tswitch solution
        slowU = RRTStar7.slowU(0, 1, 1, 1, 0.732);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(2.000, slowU.s1.u, 0.001);
        assertEquals(0.366, slowU.s1.t, 0.001);
        assertEquals(-2.000, slowU.s2.u, 0.001);
        assertEquals(0.366, slowU.s2.t, 0.001); // midpoint
        System.out.println("the correct answer is to just drift, u=0");
        slowU = RRTStar7.slowU(0, 1, 1, 1, 1.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(0, slowU.s1.u, 0.001);
        assertEquals(1.000, slowU.s1.t, 0.001);
        assertEquals(0, slowU.s2.u, 0.001);
        assertEquals(0, slowU.s2.t, 0.001); // no second segment
        // down to the axis and back, I-G+
        slowU = RRTStar7.slowU(0, 1, 1, 1, 2.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-1, slowU.s1.u, 0.001);
        assertEquals(1, slowU.s1.t, 0.001);
        assertEquals(1, slowU.s2.u, 0.001);
        assertEquals(1, slowU.s2.t, 0.001);

        // no limit or mirror at u=2
        assertEquals(1.000, RRTStar7.tSwitch(0, 1, 1.5, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1.5, 1, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1.5, 1, 2), 0.001);
        // discover tswitch, I+G-
        slowU = RRTStar7.slowU(0, 1, 1.5, 1, 1.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(2.000, slowU.s1.u, 0.001);
        assertEquals(0.5, slowU.s1.t, 0.001);
        assertEquals(-2.000, slowU.s2.u, 0.001);
        assertEquals(0.5, slowU.s2.t, 0.001); // midpoint
        // t=2, u=0.5, intersects at (0.75,0.5), I-G+
        slowU = RRTStar7.slowU(0, 1, 1.5, 1, 2.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-0.500, slowU.s1.u, 0.001);
        assertEquals(1.0, slowU.s1.t, 0.001);
        assertEquals(0.500, slowU.s2.u, 0.001);
        assertEquals(1.0, slowU.s2.t, 0.001); // midpoint
        // taking longer takes *more* u in order to slow down harder, I-G+
        slowU = RRTStar7.slowU(0, 1, 1.5, 1, 3.000);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(1, slowU.idot, 0.001);
        assertEquals(1.5, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(-0.666, slowU.s1.u, 0.001);
        assertEquals(1.5, slowU.s1.t, 0.001);
        assertEquals(0.666, slowU.s2.u, 0.001);
        assertEquals(1.5, slowU.s2.t, 0.001); // midpoint

        // a short path with limit and mirror
        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);
        assertEquals(0.585, RRTStar7.tLimit(0, 2, 1, 2, 2), 0.001);
        assertEquals(3.414, RRTStar7.tMirror(0, 2, 1, 2, 2), 0.001);
        // discover tswitch (the fast path) I+G-
        slowU = RRTStar7.slowU(0, 2, 1, 2, 0.449);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(2, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(2.023, slowU.s1.u, 0.001);
        assertEquals(0.224, slowU.s1.t, 0.001);
        assertEquals(-2.023, slowU.s2.u, 0.001);
        assertEquals(0.224, slowU.s2.t, 0.001); // midpoint
        // discover tlimit I-G+
        slowU = RRTStar7.slowU(0, 2, 1, 2, 0.585);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(2, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(-1.986, slowU.s1.u, 0.001);
        assertEquals(0.292, slowU.s1.t, 0.001);
        assertEquals(1.986, slowU.s2.u, 0.001);
        assertEquals(0.292, slowU.s2.t, 0.001); // midpoint
        // discover tmirror I-G+
        slowU = RRTStar7.slowU(0, 2, 1, 2, 3.414);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(2, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(-2.000, slowU.s1.u, 0.001);
        assertEquals(1.707, slowU.s1.t, 0.001);
        assertEquals(2.000, slowU.s2.u, 0.001);
        assertEquals(1.707, slowU.s2.t, 0.001); // midpoint
        // try a couple of times between tlimit and tmirror
        // it's possible you just need more u. I-G+
        slowU = RRTStar7.slowU(0, 2, 1, 2, 1);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(2, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(-4.000, slowU.s1.u, 0.001);
        assertEquals(0.5, slowU.s1.t, 0.001);
        assertEquals(4.000, slowU.s2.u, 0.001);
        assertEquals(0.5, slowU.s2.t, 0.001); // midpoint
        // I-G+
        slowU = RRTStar7.slowU(0, 2, 1, 2, 2);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(2, slowU.idot, 0.001);
        assertEquals(1, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(-3.000, slowU.s1.u, 0.001);
        assertEquals(1.0, slowU.s1.t, 0.001);
        assertEquals(3.000, slowU.s2.u, 0.001);
        assertEquals(1.0, slowU.s2.t, 0.001);

        // up from zero I+G-
        assertEquals(1, RRTStar7.tSwitch(1, 0, 2, 2, 2), 0.001);
        slowU = RRTStar7.slowU(1, 0, 2, 2, 1);
        assertEquals(1, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(2, slowU.g, 0.001);
        assertEquals(2, slowU.gdot, 0.001);
        assertEquals(2.0, slowU.s1.u, 0.001);
        assertEquals(1.0, slowU.s1.t, 0.001);
        assertEquals(-2.0, slowU.s2.u, 0.001);
        assertEquals(0.0, slowU.s2.t, 0.001); // no second segment

        // from negative idot, cross the axis, switch. I+G-
        assertEquals(2.645, RRTStar7.tSwitch(0, -1, 3, 1, 2), 0.001);
        // discover tswitch
        slowU = RRTStar7.slowU(0, -1, 3, 1, 2.645);
        assertEquals(0, slowU.i, 0.001);
        assertEquals(-1, slowU.idot, 0.001);
        assertEquals(3, slowU.g, 0.001);
        assertEquals(1, slowU.gdot, 0.001);
        assertEquals(2.000, slowU.s1.u, 0.001);
        assertEquals(1.822, slowU.s1.t, 0.001);
        assertEquals(-2.000, slowU.s2.u, 0.001);
        assertEquals(0.822, slowU.s2.t, 0.001);
    }

    @Test
    void testSlowU2() {
        // chasing a case.
        Axis slowU = RRTStar7.slowU(15.5, 0, 15.548, -1.633, 1.6);
        assertEquals(15.5, slowU.i, 0.001);
        assertEquals(0, slowU.idot, 0.001);
        assertEquals(15.548, slowU.g, 0.001);
        assertEquals(-1.633, slowU.gdot, 0.001);
        assertEquals(2.528, slowU.s1.u, 0.001);
        assertEquals(0.477, slowU.s1.t, 0.001);
        assertEquals(-2.528, slowU.s2.u, 0.001);
        assertEquals(1.122, slowU.s2.t, 0.001);
    }

    @Test
    void testTSwitch() {

        assertEquals(0.724, RRTStar7.tSwitch(0, 0, 0.5, 1.0, 2), 0.001);

        // there are three solutions to this case; tSwitch returns the fastest.
        // see below for tLimit and tMirror.
        assertEquals(0.414, RRTStar7.tSwitch(0, 1, 0.5, 1, 2), 0.001);
        // the fast way
        assertEquals(0.732, RRTStar7.tSwitch(0, 1, 1, 1, 2), 0.001);
        assertEquals(1.000, RRTStar7.tSwitch(0, 1, 1.5, 1, 2), 0.001);

        // same story here, we choose the slow path but there's also a fast one.
        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);
        assertEquals(1.162, RRTStar7.tSwitch(0, 2, 3, 2, 2), 0.001);
        assertEquals(1.742, RRTStar7.tSwitch(0, 2, 5, 2, 2), 0.001);

        // no movement == zero time
        assertEquals(0, RRTStar7.tSwitch(0, 0, 0, 0, 2.5), 0.001);

        // simply move back, rest-to-rest
        // (1,0) -> (0,0) takes 1.264
        assertEquals(1.264, RRTStar7.tSwitch(1, 0, 0, 0, 2.5), 0.001);

        // slows to a stop, back, stop, forward, motion-to-motion
        assertEquals(2.759, RRTStar7.tSwitch(1, 1, -1, 1, 2.5), 0.001);

        // (-1,1) -> (0,0), used below
        assertEquals(0.985, RRTStar7.tSwitch(-1, 1, 0, 0, 2.5), 0.001);

        // (0,0) -> (1,0), used below
        assertEquals(1.264, RRTStar7.tSwitch(0, 0, 1, 0, 2.5), 0.001);
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 0, 1, 0, 2.5), 0.001);

        // chasing a case i missed
        // somehow it thinks that the I+G- path in reverse is viable. how does it choose
        // it?
        assertEquals(Double.NaN, RRTStar7.tSwitchIplusGminus(7.55, 2.1818, 6.75, 0, 2.5), 0.001);
        // this is what it *should* choose
        assertEquals(2.547, RRTStar7.tSwitchIminusGplus(7.55, 2.1818, 6.75, 0, 2.5), 0.001);
        // ok fixed
        assertEquals(2.547, RRTStar7.tSwitch(7.55, 2.1818, 6.75, 0, 2.5), 0.001);

    }

    @Test
    void testIntercepts() {
        assertEquals(0, RRTStar7.c_minus(0, 0, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(0, 0, 1), 0.001);

        assertEquals(0.5, RRTStar7.c_minus(0, 1, 1), 0.001);
        assertEquals(-0.5, RRTStar7.c_plus(0, 1, 1), 0.001);

        assertEquals(1.5, RRTStar7.c_minus(1, 1, 1), 0.001);
        assertEquals(0.5, RRTStar7.c_plus(1, 1, 1), 0.001);

        assertEquals(-0.5, RRTStar7.c_minus(-1, 1, 1), 0.001);
        assertEquals(-1.5, RRTStar7.c_plus(-1, 1, 1), 0.001);

        assertEquals(0.5, RRTStar7.c_minus(0, -1, 1), 0.001);
        assertEquals(-0.5, RRTStar7.c_plus(0, -1, 1), 0.001);

        assertEquals(2, RRTStar7.c_minus(0, 2, 1), 0.001);
        assertEquals(-2, RRTStar7.c_plus(0, 2, 1), 0.001);

        assertEquals(0.25, RRTStar7.c_minus(0, 1, 2), 0.001);
        assertEquals(-0.25, RRTStar7.c_plus(0, 1, 2), 0.001);

        // these are cases for the switching point test below
        // these curves don't intersect at all
        assertEquals(-1, RRTStar7.c_minus(-3, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);

        // these curves intersect exactly once at the origin
        assertEquals(0, RRTStar7.c_minus(-2, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);

        // these two curves intersect twice, once at (0.5,1) and once at (0.5,-1)
        assertEquals(1, RRTStar7.c_minus(-1, 2, 1), 0.001);
        assertEquals(0, RRTStar7.c_plus(2, 2, 1), 0.001);
    }

    @Test
    void testQSwitch() {
        assertEquals(0.375, RRTStar7.qSwitchIplusGminus(0, 0, 0.5, 1.0, 2), 0.001);
        assertEquals(0.125, RRTStar7.qSwitchIminusGplus(0, 0, 0.5, 1.0, 2), 0.001);

        assertEquals(-0.5, RRTStar7.qSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(0, RRTStar7.qSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-0.5, RRTStar7.qSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(0.0, RRTStar7.qSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);

        // these are all a little different just to avoid zero as the answer
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(2, 2, -1, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIplusGminus(-1, 2, 2, -2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(2, 2, -1, 2, 1), 0.001);
        assertEquals(0.5, RRTStar7.qSwitchIminusGplus(-1, 2, 2, -2, 1), 0.001);
    }

    @Test
    void testQDotSwitch() {
        assertEquals(1.224, RRTStar7.qDotSwitchIplusGminus(0, 0, 0.5, 1.0, 2), 0.001);
        assertEquals(Double.NaN, RRTStar7.qDotSwitchIminusGplus(0, 0, 0.5, 1.0, 2), 0.001);

        assertEquals(3.000, RRTStar7.qDotSwitchIplusGminus(-3, 2, 2, 2, 1), 0.001);
        assertEquals(2.828, RRTStar7.qDotSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        assertEquals(2.645, RRTStar7.qDotSwitchIplusGminus(-1, 2, 2, 2, 1), 0.001);

        assertEquals(-3.0, RRTStar7.qDotSwitchIminusGplus(2, -2, -3, -2, 1), 0.001);
        assertEquals(-2.828, RRTStar7.qDotSwitchIminusGplus(2, -2, -2, -2, 1), 0.001);
        assertEquals(-2.645, RRTStar7.qDotSwitchIminusGplus(2, -2, -1, -2, 1), 0.001);

        // from 2,2 to -2,2, I+G- is invalid
        assertEquals(0, RRTStar7.qDotSwitchIplusGminus(2, 2, -2, 2, 1), 0.001);
        // from -2,2 to 2,-2 switches in the same place as -2,2->2,2
        assertEquals(2.828, RRTStar7.qDotSwitchIplusGminus(-2, 2, 2, -2, 1), 0.001);
        // from 2,2 to -2,2 switches at the bottom
        assertEquals(-2.828, RRTStar7.qDotSwitchIminusGplus(2, 2, -2, 2, 1), 0.001);
        // from -2,2 to 2,-2, I-G+ is invalid
        assertEquals(0, RRTStar7.qDotSwitchIminusGplus(-2, 2, 2, -2, 1), 0.001);
    }

    @Test
    void testTSwitchByPath() {
        // from -2 to 2, the 'fast' and normal way
        assertEquals(1.656, RRTStar7.tSwitchIplusGminus(-2, 2, 2, 2, 1), 0.001);
        // this path goes from (-2,2) to (0,0) and then to (2,2)
        assertEquals(4.000, RRTStar7.tSwitchIminusGplus(-2, 2, 2, 2, 1), 0.001);

        // the opposite order, 2 to -2, this is a completely invalid result,
        // traversing Iplus backwards, and then Gminus backwards.
        assertEquals(Double.NaN, RRTStar7.tSwitchIplusGminus(2, 2, -2, 2, 1), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(9.656, RRTStar7.tSwitchIminusGplus(2, 2, -2, 2, 1), 0.001);

        // diagonal, the 'fast' and normal way
        assertEquals(5.656, RRTStar7.tSwitchIplusGminus(-2, 2, 2, -2, 1), 0.001);
        // this is completely invalid
        assertEquals(Double.NaN, RRTStar7.tSwitchIminusGplus(-2, 2, 2, -2, 1), 0.001);
        // this is invalid, it should yield like NaN or something
        assertEquals(Double.NaN, RRTStar7.tSwitchIplusGminus(2, -2, -2, 2, 1), 0.001);
        // from 2 to -2 is the 'long way around' across the x-axis and back.
        assertEquals(5.656, RRTStar7.tSwitchIminusGplus(2, -2, -2, 2, 1), 0.001);

        // another problem case
        // this can't be done
        assertEquals(Double.NaN, RRTStar7.tSwitchIminusGplus(-1, 1, 0, 0, 1), 0.001);
        // the "normal" way
        assertEquals(1.449, RRTStar7.tSwitchIplusGminus(-1, 1, 0, 0, 1), 0.001);

    }

    @Test
    void testNearest() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), T_a, T_b);

        solver.setRadius(10);

        // add a node
        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 })));
        System.out.println(T_a);

        // look for it
        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 0, 0 }), T_a,
                true);

        System.out.println(near);
        // find it: note this the time, not the Euclidean distance
        // (1,0) to (0,0) takes 1.264 (see above)
        assertEquals(1.264, near._dist, 0.001);
        assertArrayEquals(new double[] { 0, 0, 0, 0 }, near._nearest.getState().getData(), 0.001);
    }

    @Test
    void testNearest2() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), T_a, T_b);
        solver.setRadius(10);

        KDTree.insert(arena, T_a, new Node<>(new Matrix<>(Nat.N4(), Nat.N1(), new double[] { -1, 1, 0, 0 })));
        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 1, 0, 0 }), T_a,
                true);
        // (-1,1) to (0,0)
        assertEquals(1.159, near._dist, 0.001);
        assertArrayEquals(new double[] { -1, 1, 0, 0 }, near._nearest.getState().getData(), 0.001);
    }

    @Test
    void testNearest3() {
        /**
         * _init = { 15.5, 0, 6.75, 0 });
         * _goal = { 1.93, 0, 2.748, 0 });
         */
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), T_a, T_b);

        // note small radius; this won't find anything
        solver.setRadius(1);

        System.out.println(T_a);

        KDNearNode<Node<N4>> near = solver.BangBangNearest(
                new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 0, 0 }), T_a,
                true);
        // this should find nothing
        assertNull(near);
    }

    static Matrix<N4, N1> s(double x1, double x2, double x3, double x4) {
        return new Matrix<>(Nat.N4(), Nat.N1(), new double[] { x1, x2, x3, x4 });
    }

    @Test
    void testOptimalTrajectory() {
        // same cases as below.
        // symmetrical, diagonal
        Trajectory t = RRTStar7.optimalTrajectory(s(0, 0, 0, 0), s(1, 0, 1, 0), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(0, t.x.idot, 0.001);
        assertEquals(1, t.x.g, 0.001);
        assertEquals(0, t.x.gdot, 0.001);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.632, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(0.632, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(0, t.y.idot, 0.001);
        assertEquals(1, t.y.g, 0.001);
        assertEquals(0, t.y.gdot, 0.001);
        assertEquals(2.5, t.y.s1.u, 0.001);
        assertEquals(0.632, t.y.s1.t, 0.001);
        assertEquals(-2.5, t.y.s2.u, 0.001);
        assertEquals(0.632, t.y.s2.t, 0.001);

        // x needs to be slowed at part-throttle while y goes full
        // both switch at the same time, in the middle.
        // this makes an S shape.
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 0), s(0.5, 1, 1, 0), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(1, t.x.idot, 0.001);
        assertEquals(0.5, t.x.g, 0.001);
        assertEquals(1, t.x.gdot, 0.001);
        assertEquals(-1.912, t.x.s1.u, 0.001);
        assertEquals(0.632, t.x.s1.t, 0.001);
        assertEquals(1.912, t.x.s2.u, 0.001);
        assertEquals(0.632, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(0, t.y.idot, 0.001);
        assertEquals(1, t.y.g, 0.001);
        assertEquals(0, t.y.gdot, 0.001);
        assertEquals(2.5, t.y.s1.u, 0.001);
        assertEquals(0.632, t.y.s1.t, 0.001);
        assertEquals(-2.5, t.y.s2.u, 0.001);
        assertEquals(0.632, t.y.s2.t, 0.001);

        // x states are very close together
        // y is the same as above
        // the y fast path is in the x gap, so y goes a little slower than optimal
        // (without braking) to match x mirror.
        // this makes a more exaggerated S shape
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 0), s(0.25, 1, 1, 0), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(1, t.x.idot, 0.001);
        assertEquals(0.25, t.x.g, 0.001);
        assertEquals(1, t.x.gdot, 0.001);
        assertEquals(-2.5, t.x.s1.u, 0.001);
        assertEquals(0.644, t.x.s1.t, 0.001);
        assertEquals(2.5, t.x.s2.u, 0.001);
        assertEquals(0.644, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(0, t.y.idot, 0.001);
        assertEquals(1, t.y.g, 0.001);
        assertEquals(0, t.y.gdot, 0.001);
        assertEquals(2.404, t.y.s1.u, 0.001);
        assertEquals(0.644, t.y.s1.t, 0.001);
        assertEquals(-2.404, t.y.s2.u, 0.001);
        assertEquals(0.644, t.y.s2.t, 0.001);

        // both x and y states are very close together.
        // as above the y fast path is in the x gap
        // but the y points are so close together that y needs to brake to match x
        // mirror.
        t = RRTStar7.optimalTrajectory(s(0, 1, 0, 1), s(0.25, 1, 0.5, 1), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(1, t.x.idot, 0.001);
        assertEquals(0.25, t.x.g, 0.001);
        assertEquals(1, t.x.gdot, 0.001);
        assertEquals(-2.5, t.x.s1.u, 0.001);
        assertEquals(0.644, t.x.s1.t, 0.001);
        assertEquals(2.5, t.x.s2.u, 0.001);
        assertEquals(0.644, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(1, t.y.idot, 0.001);
        assertEquals(0.5, t.y.g, 0.001);
        assertEquals(1, t.y.gdot, 0.001);
        assertEquals(-1.898, t.y.s1.u, 0.001);
        assertEquals(0.644, t.y.s1.t, 0.001);
        assertEquals(1.898, t.y.s2.u, 0.001);
        assertEquals(0.644, t.y.s2.t, 0.001);

        // this shows that a diagonal not at 45 degrees involves u values
        // proportional to the angle.
        // it's just a straight line that speeds up in the middle.
        t = RRTStar7.optimalTrajectory(s(0, 2, 0, 1), s(1, 2, 0.5, 1), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(2, t.x.idot, 0.001);
        assertEquals(1, t.x.g, 0.001);
        assertEquals(2, t.x.gdot, 0.001);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.219, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(0.219, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(1, t.y.idot, 0.001);
        assertEquals(0.5, t.y.g, 0.001);
        assertEquals(1, t.y.gdot, 0.001);
        assertEquals(1.25, t.y.s1.u, 0.001);
        assertEquals(0.219, t.y.s1.t, 0.001);
        assertEquals(-1.25, t.y.s2.u, 0.001);
        assertEquals(0.219, t.y.s2.t, 0.001);

        // here's a longer-range case. long-range cases are easier.
        // ahead x, go to a far-away spot and stop.
        // since there's initial velocity, the braking phase is longer.
        // this is also an example where the switching points
        // of the axes are not the same.
        // TODO: velocity limit for these cases
        t = RRTStar7.optimalTrajectory(s(0, 5, 0, 0), s(10, 0, 5, 0), 2.5);
        assertEquals(0, t.x.i, 0.001);
        assertEquals(5, t.x.idot, 0.001);
        assertEquals(10, t.x.g, 0.001);
        assertEquals(0, t.x.gdot, 0.001);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.449, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(2.449, t.x.s2.t, 0.001);
        assertEquals(0, t.y.i, 0.001);
        assertEquals(0, t.y.idot, 0.001);
        assertEquals(5, t.y.g, 0.001);
        assertEquals(0, t.y.gdot, 0.001);
        assertEquals(2.379, t.y.s1.u, 0.001);
        assertEquals(1.449, t.y.s1.t, 0.001);
        assertEquals(-2.379, t.y.s2.u, 0.001);
        assertEquals(1.449, t.y.s2.t, 0.001);
    }

    @Test
    void testOptimalTrajectory2() {
        // chasing a case
        Trajectory t = RRTStar7.optimalTrajectory(s(15.5, 0, 6.75, 0), s(15.548, -1.633, 6.303, 0.663), 2.5);
        assertEquals(15.5, t.x.i, 0.001);
        assertEquals(0, t.x.idot, 0.001);
        assertEquals(15.548, t.x.g, 0.001);
        assertEquals(-1.633, t.x.gdot, 0.001);
        assertEquals(2.5, t.x.s1.u, 0.001);
        assertEquals(0.482, t.x.s1.t, 0.001);
        assertEquals(-2.5, t.x.s2.u, 0.001);
        assertEquals(1.135, t.x.s2.t, 0.001);
        assertEquals(6.75, t.y.i, 0.001);
        assertEquals(0, t.y.idot, 0.001);
        assertEquals(6.303, t.y.g, 0.001);
        assertEquals(0.663, t.y.gdot, 0.001);
        assertEquals(-1.607, t.y.s1.u, 0.001);
        assertEquals(0.602, t.y.s1.t, 0.001);
        assertEquals(1.607, t.y.s2.u, 0.001);
        assertEquals(1.015, t.y.s2.t, 0.001);
    }

    @Test
    void testTOptimal() {
        // no movement = no time

        // same trajectory in both axes
        // x: (0,0) -> (1,0)
        // y: (0,0) -> (1,0)
        assertEquals(1.264, RRTStar7.tOptimal(s(0, 0, 0, 0), s(1, 0, 1, 0), 2.5), 0.001);

        // slow one is slower than tmirror
        // x: (0,1) -> (0.5,1): tswitch = 0.414, tlimit=tmirror=1
        // y: (0,0) -> (1,0)
        assertEquals(1.264, RRTStar7.tOptimal(s(0, 1, 0, 0), s(0.5, 1, 1, 0), 2.5), 0.001);

        // the y axis is in the gap of the x axis, so use tmirror
        // x: (0,1) -> (0.25,1) tswitch=0.224 tlimit=0.292 tmirror=1.707
        // y: (0,0) -> (1.00,0) tswitch=1.264 tlimit=nan tmirror=nan
        // note u = 2
        assertEquals(1.707, RRTStar7.tOptimal(s(0, 1, 0, 0), s(0.25, 1, 1, 0), 2.0), 0.001);

        // another gap example
        // x: (0,1) -> (0.25,1) tswitch=0.224 tlimit=0.292 tmirror=1.707
        // y: (0,1) -> (0.50,1) tswitch=0.414 tlimit=1.000 tmirror=1.000
        // note u = 2
        assertEquals(1.707, RRTStar7.tOptimal(s(0, 1, 0, 1), s(0.25, 1, 0.5, 1), 2.0), 0.001);

        // both are above the gap so it just picks the slower tswitch
        // x: (0,2) -> (1.0,2) tswitch=0.449 tlimit=0.585 tmirror=3.414
        // y: (0,1) -> (0.5,1) tswitch=0.414 tlimit=1.000 tmirror=1.000
        assertEquals(0.449, RRTStar7.tOptimal(s(0, 2, 0, 1), s(1, 2, 0.5, 1), 2.0), 0.001);
    }

    @Test
    void testTOptimal2() {
        assertEquals(1.617, RRTStar7.tOptimal(s(15.5, 0, 6.75, 0), s(15.548, -1.633, 6.303, 0.663), 2.5), 0.001);

        RRTStar7.DEBUG = true;
        Axis a = RRTStar7.slowU(13.491, -0.413, 13.857, 1.799, 1.297);
        System.out.println(a);
        Trajectory trajectory = RRTStar7.optimalTrajectory(s(13.491, -0.413, 7.168, 0.079),
                s(13.857, 1.799, 6.908, -1.644), 2.5);
        System.out.println(trajectory);

        assertEquals(1.297, RRTStar7.tOptimal(s(13.491, -0.413, 7.168, 0.079), s(13.857, 1.799, 6.908, -1.644), 2.5),
                0.001);

    }

    // these four cases are four inversions of the same simple case.

    @Test
    void testSlowU3() {
        /*
         * a 4.326400 b 4.160000 c -9.000000
         * plus [1.0395565673886438, -2.0010950289271054] minus [2.0010950289271054,
         * -1.0395565673886438]
         * p 1.039557 ts 2.482923
         * reject ts > tw
         * p -2.001095 ts 0.290410
         * reject p < 0
         * m 2.001095 ts 1.789590
         * accept m 2.001095
         * m -1.039557 ts -0.402923
         * reject m < 0
         */
        RRTStar7.DEBUG = true;
        Axis a = RRTStar7.slowU(0, -1, 0, 2, 2.08);
        assertEquals(-2.001, a.s1.u, 0.001);
        assertEquals(0.29, a.s1.t, 0.001);
        assertEquals(2.001, a.s2.u, 0.001);
        assertEquals(1.79, a.s2.t, 0.001);
    }

    @Test
    void testSlowU4() {
        /*
         * a 4.326400 b 4.160000 c -9.000000
         * plus [1.0395565673886438, -2.0010950289271054] minus [2.0010950289271054,
         * -1.0395565673886438]
         * p 1.039557 ts -0.402923
         * reject ts < 0
         * p -2.001095 ts 1.789590
         * reject p < 0
         * m 2.001095 ts 0.290410
         * accept m 2.001095
         * m -1.039557 ts 2.482923
         * reject m < 0
         */
        RRTStar7.DEBUG = true;
        Axis a = RRTStar7.slowU(0, 2, 0, -1, 2.08);
        assertEquals(-2.001, a.s1.u, 0.001);
        assertEquals(1.79, a.s1.t, 0.001);
        assertEquals(2.001, a.s2.u, 0.001);
        assertEquals(0.29, a.s2.t, 0.001);
    }

    @Test
    void testSlowU5() {
        /*
         * a 4.326400 b -4.160000 c -9.000000
         * plus [2.0010950289271054, -1.0395565673886438] minus [1.0395565673886438,
         * -2.0010950289271054]
         * p 2.001095 ts 0.290410
         * accept p 2.001095
         * p -1.039557 ts 2.482923
         * reject p < 0
         * m 1.039557 ts -0.402923
         * reject ts < 0
         * m -2.001095 ts 1.789590
         * reject m < 0
         */
        RRTStar7.DEBUG = true;
        Axis a = RRTStar7.slowU(0, 1, 0, -2, 2.08);
        assertEquals(2.001, a.s1.u, 0.001);
        assertEquals(0.29, a.s1.t, 0.001);
        assertEquals(-2.001, a.s2.u, 0.001);
        assertEquals(1.79, a.s2.t, 0.001);
    }

    @Test
    void testSlowU6() {
        /*
         * a 4.326400 b -4.160000 c -9.000000
         * plus [2.0010950289271054, -1.0395565673886438] minus [1.0395565673886438,
         * -2.0010950289271054]
         * p 2.001095 ts 1.789590
         * accept p 2.001095
         * p -1.039557 ts -0.402923
         * reject p < 0
         * m 1.039557 ts 2.482923
         * reject ts > tw
         * m -2.001095 ts 0.290410
         * reject m < 0
         */
        RRTStar7.DEBUG = true;
        Axis a = RRTStar7.slowU(0, -2, 0, 1, 2.08);
        assertEquals(2.001, a.s1.u, 0.001);
        assertEquals(1.79, a.s1.t, 0.001);
        assertEquals(-2.001, a.s2.u, 0.001);
        assertEquals(0.29, a.s2.t, 0.001);
    }

    @Test
    void testTSwitch2() {
        // about 0.5s to switching, 0.5s back to axis, 0.6s or so to goal.
        assertEquals(1.617, RRTStar7.tSwitch(15.5, 0, 15.548, -1.633, 2.5), 0.001);
        assertEquals(1.617, RRTStar7.tSwitchIplusGminus(15.5, 0, 15.548, -1.633, 2.5), 0.001);
        assertEquals(1.205, RRTStar7.qDotSwitchIplusGminus(15.5, 0, 15.548, -1.633, 2.5), 0.001);
        // this path is impossible.
        assertEquals(Double.NaN, RRTStar7.tSwitchIminusGplus(15.5, 0, 15.548, -1.633, 2.5), 0.001);
        assertEquals(-1.101, RRTStar7.qDotSwitchIminusGplus(15.5, 0, 15.548, -1.633, 2.5), 0.001);
    }

    @Test
    void testTLimit2() {
        assertEquals(Double.NaN, RRTStar7.tLimit(15.5, 0, 15.548, -1.633, 2.5), 0.001);
    }

    @Test
    void testTMirror2() {
        assertEquals(Double.NaN, RRTStar7.tMirror(15.5, 0, 15.548, -1.633, 2.5), 0.001);
    }

    @Test
    void testNaN() {
        // here's where i learned that sometimes these timing functions produce
        // "negative zero" which
        // fails a <0 test even though it's actually zero. sigh.
        assertEquals(0.3, RRTStar7.tSwitchIplusGminus(2.698095, 0.577782, 2.758929, -0.172218, 2.5), 0.001);
        assertEquals(0.3, RRTStar7.tSwitchIminusGplus(2.698095, 0.577782, 2.758929, -0.172218, 2.5), 0.001);
        assertEquals(0.3, RRTStar7.tSwitch(2.698095, 0.577782, 2.758929, -0.172218, 2.5), 0.001);

        assertEquals(0.841, RRTStar7.tSwitchIplusGminus(8.635502, 1.556397, 9.060257, -0.546445, 2.500000), 0.001);
        assertEquals(0.841, RRTStar7.tSwitchIminusGplus(8.635502, 1.556397, 9.060257, -0.546445, 2.500000), 0.001);
        assertEquals(0.841, RRTStar7.tSwitch(8.635502, 1.556397, 9.060257, -0.546445, 2.500000), 0.001);
    }

    @Test
    void testSampleAxis() {
        // case from elsewhere
        // (1,0) -> (0,0) takes 1.264
        // assertEquals(1.264, RRTStar7.tSwitch(1, 0, 0, 0, 2.5), 0.001);
        Axis a = new Axis();
        a.i = 1;
        a.idot = 0;
        a.g = 0;
        a.gdot = 0;
        a.s1.u = -2.5;
        a.s1.t = 0.632;
        a.s2.u = 2.5;
        a.s2.t = 0.632;
        assertArrayEquals(new double[] { 1.000, 0.000 }, RRTStar7.SampleAxis(a, 0).getData(), 0.001);
        assertArrayEquals(new double[] { 0.980, -0.312 }, RRTStar7.SampleAxis(a, 0.125).getData(), 0.001);
        assertArrayEquals(new double[] { 0.921, -0.625 }, RRTStar7.SampleAxis(a, 0.25).getData(), 0.001);
        assertArrayEquals(new double[] { 0.824, -0.937 }, RRTStar7.SampleAxis(a, 0.375).getData(), 0.001);
        assertArrayEquals(new double[] { 0.687, -1.25 }, RRTStar7.SampleAxis(a, 0.5).getData(), 0.001);
        assertArrayEquals(new double[] { 0.511, -1.562 }, RRTStar7.SampleAxis(a, 0.625).getData(), 0.001);
        assertArrayEquals(new double[] { 0.330, -1.285 }, RRTStar7.SampleAxis(a, 0.75).getData(), 0.001);
        assertArrayEquals(new double[] { 0.189, -0.972 }, RRTStar7.SampleAxis(a, 0.875).getData(), 0.001);
        assertArrayEquals(new double[] { 0.087, -0.66 }, RRTStar7.SampleAxis(a, 1).getData(), 0.001);
        assertArrayEquals(new double[] { 0.024, -0.347 }, RRTStar7.SampleAxis(a, 1.125).getData(), 0.001);
        assertArrayEquals(new double[] { 0.000, -0.035 }, RRTStar7.SampleAxis(a, 1.25).getData(), 0.001);
        assertArrayEquals(new double[] { 0.000, 0.000 }, RRTStar7.SampleAxis(a, 1.264).getData(), 0.001);
    }

    @Test
    void testSampleAxis2() {
        // x: (0,1) -> (0.5,1): tswitch = 0.414, tlimit=tmirror=1
        // this is tswitch I+G-
        Axis a = new Axis();
        a.i = 0;
        a.idot = 1;
        a.g = 0.5;
        a.gdot = 1;
        a.s1.u = 2;
        a.s1.t = 0.207;
        a.s2.u = -2;
        a.s2.t = 0.207;
        assertArrayEquals(new double[] { 0.000, 1.000 }, RRTStar7.SampleAxis(a, 0).getData(), 0.001);
        assertArrayEquals(new double[] { 0.110, 1.2 }, RRTStar7.SampleAxis(a, 0.1).getData(), 0.001);
        assertArrayEquals(new double[] { 0.240, 1.4 }, RRTStar7.SampleAxis(a, 0.2).getData(), 0.001);
        assertArrayEquals(new double[] { 0.373, 1.228 }, RRTStar7.SampleAxis(a, 0.3).getData(), 0.001);
        assertArrayEquals(new double[] { 0.485, 1.028 }, RRTStar7.SampleAxis(a, 0.4).getData(), 0.001);
        assertArrayEquals(new double[] { 0.5, 1 }, RRTStar7.SampleAxis(a, 0.5).getData(), 0.001);
    }

    @Test
    void testSampleAxis3() {
        // x: (0,1) -> (0.5,1): tswitch = 0.414, tlimit=tmirror=1
        // same as above
        // this is tmirror I-G+
        Axis a = new Axis();
        a.i = 0;
        a.idot = 1;
        a.g = 0.5;
        a.gdot = 1;
        a.s1.u = -2;
        a.s1.t = 0.5;
        a.s2.u = 2;
        a.s2.t = 0.5;
        assertArrayEquals(new double[] { 0.00, 1.0 }, RRTStar7.SampleAxis(a, 0).getData(), 0.001);
        assertArrayEquals(new double[] { 0.09, 0.8 }, RRTStar7.SampleAxis(a, 0.1).getData(), 0.001);
        assertArrayEquals(new double[] { 0.16, 0.6 }, RRTStar7.SampleAxis(a, 0.2).getData(), 0.001);
        assertArrayEquals(new double[] { 0.21, 0.4 }, RRTStar7.SampleAxis(a, 0.3).getData(), 0.001);
        assertArrayEquals(new double[] { 0.24, 0.2 }, RRTStar7.SampleAxis(a, 0.4).getData(), 0.001);
        assertArrayEquals(new double[] { 0.25, 0.0 }, RRTStar7.SampleAxis(a, 0.5).getData(), 0.001);
        assertArrayEquals(new double[] { 0.26, 0.2 }, RRTStar7.SampleAxis(a, 0.6).getData(), 0.001);
        assertArrayEquals(new double[] { 0.29, 0.4 }, RRTStar7.SampleAxis(a, 0.7).getData(), 0.001);
        assertArrayEquals(new double[] { 0.34, 0.6 }, RRTStar7.SampleAxis(a, 0.8).getData(), 0.001);
        assertArrayEquals(new double[] { 0.41, 0.8 }, RRTStar7.SampleAxis(a, 0.9).getData(), 0.001);
        assertArrayEquals(new double[] { 0.50, 1.0 }, RRTStar7.SampleAxis(a, 1).getData(), 0.001);
    }

    @Test
    void testSampleAxis4() {
        // reflection
        // x: (0,-1) -> (-0.5,-1): tswitch = 0.414, tlimit=tmirror=1
        // this is tswitch I+G-
        Axis a = new Axis();
        a.i = 0;
        a.idot = -1;
        a.g = -0.5;
        a.gdot = -1;
        a.s1.u = -2;
        a.s1.t = 0.207;
        a.s2.u = 2;
        a.s2.t = 0.207;
        assertArrayEquals(new double[] { 0.000, -1.000 }, RRTStar7.SampleAxis(a, 0).getData(), 0.001);
        assertArrayEquals(new double[] { -0.110, -1.2 }, RRTStar7.SampleAxis(a, 0.1).getData(), 0.001);
        assertArrayEquals(new double[] { -0.240, -1.4 }, RRTStar7.SampleAxis(a, 0.2).getData(), 0.001);
        assertArrayEquals(new double[] { -0.373, -1.228 }, RRTStar7.SampleAxis(a, 0.3).getData(), 0.001);
        assertArrayEquals(new double[] { -0.485, -1.028 }, RRTStar7.SampleAxis(a, 0.4).getData(), 0.001);
        assertArrayEquals(new double[] { -0.5, -1 }, RRTStar7.SampleAxis(a, 0.5).getData(), 0.001);
    }

    @Test
    void testSteerFree() {
        Predicate<Matrix<N4, N1>> free = new Predicate<>() {
            @Override
            public boolean test(Matrix<N4, N1> config) {
                return true;
            }
        };
        // x: (0,0) -> (1,0)
        // y: (0,0) -> (1,0)
        Matrix<N4, N1> x_i = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 });
        Matrix<N4, N1> x_g = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 1, 0 });
        Trajectory phi = RRTStar7.BangBangSteer(free, x_i, x_g, true, false);
        assertEquals(0, phi.x.i, 0.001);
        assertEquals(0, phi.x.idot, 0.001);
        assertEquals(1, phi.x.g, 0.001);
        assertEquals(0, phi.x.gdot, 0.001);
        assertEquals(2.5, phi.x.s1.u, 0.001);
        assertEquals(0.632, phi.x.s1.t, 0.001);
        assertEquals(-2.5, phi.x.s2.u, 0.001);
        assertEquals(0.632, phi.x.s2.t, 0.001);
        assertEquals(0, phi.y.i, 0.001);
        assertEquals(0, phi.y.idot, 0.001);
        assertEquals(1, phi.y.g, 0.001);
        assertEquals(0, phi.y.gdot, 0.001);
        assertEquals(2.5, phi.y.s1.u, 0.001);
        assertEquals(0.632, phi.y.s1.t, 0.001);
        assertEquals(-2.5, phi.y.s2.u, 0.001);
        assertEquals(0.632, phi.y.s2.t, 0.001);
    }

    @Test
    void testSteerObstructed() {
        Obstacle obstacle = new Polygon(Color.BLACK, 0.25, 0.25, 0.75, 0.25, 0.75, 0.75, 0.25, 0.75);
        Predicate<Matrix<N4, N1>> free = new Predicate<>() {
            @Override
            public boolean test(Matrix<N4, N1> config) {
                if (obstacle.distToPoint(config.get(0, 0), config.get(2, 0)) < 0.1)
                    return false;
                return true;

            }
        };
        // x: (0,0) -> (1,0)
        // y: (0,0) -> (1,0)
        Matrix<N4, N1> x_i = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 0, 0, 0, 0 });
        Matrix<N4, N1> x_g = new Matrix<>(Nat.N4(), Nat.N1(), new double[] { 1, 0, 1, 0 });
        Trajectory phi = RRTStar7.BangBangSteer(free, x_i, x_g, true, true);
        assertNull(phi);
    }

    @Test
    void testTLimit() {
        // tLimit does not exist when there is no intersection.
        // this is invalid
        assertEquals(-1.732, RRTStar7.qDotLimitIplusGminus(0, 1, 1, 1, 2), 0.001);
        // the slow path doesn't exist
        assertEquals(Double.NaN, RRTStar7.qDotLimitIminusGplus(0, 1, 1, 1, 2), 0.001);
        // so this should also not exist
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 1, 1, 1, 2), 0.001);

        // tLimit exists when there are both I+G- and I-G+ solutions
        // (0,1) -> (0.5,1) I-G+ intersect at one point not two, so
        // tLimit and tMirror should be the same.
        // this is invalid, it's the fast path
        assertEquals(-1.414, RRTStar7.qDotLimitIplusGminus(0, 1, 0.5, 1, 2), 0.001);
        // the single intersection is on the x axis
        assertEquals(0.000, RRTStar7.qDotLimitIminusGplus(0, 1, 0.5, 1, 2), 0.001);
        // 0.5s to the axis, 0.5s back up to g
        assertEquals(1.000, RRTStar7.tLimit(0, 1, 0.5, 1, 2), 0.001);

        // (0,1) -> (0.25,1) has two intersections.
        // this is invalid for tLimit, it's the fast path
        assertEquals(-1.224, RRTStar7.qDotLimitIplusGminus(0, 1, 0.25, 1, 2), 0.001);
        // the slow path goes a little slower than the initial state
        assertEquals(0.707, RRTStar7.qDotLimitIminusGplus(0, 1, 0.25, 1, 2), 0.001);
        // the states are nearby, doesn't take long
        assertEquals(0.292, RRTStar7.tLimit(0, 1, 0.25, 1, 2), 0.001);
        // for comparison, this is tSwitch, a little faster
        assertEquals(0.224, RRTStar7.tSwitch(0, 1, 0.25, 1, 2), 0.001);

        // (0,2) -> (1,2)
        // I+G- yields the fast path
        assertEquals(-2.449, RRTStar7.qDotLimitIplusGminus(0, 2, 1, 2, 2), 0.001);
        // I-G+ is the slow path, slower than intial/goal speed
        assertEquals(1.414, RRTStar7.qDotLimitIminusGplus(0, 2, 1, 2, 2), 0.001);
        // time to traverse the slow path
        assertEquals(0.585, RRTStar7.tLimit(0, 2, 1, 2, 2), 0.001);
        // for comparison, this is tSwitch, a little faster.
        assertEquals(0.449, RRTStar7.tSwitch(0, 2, 1, 2, 2), 0.001);

        // there are no cases here with i and g on opposite sides
        // of the qdot=0 axis, because there's no "limit" or "mirror"
        // possible in that case

        // (0,0) -> (1,0)
        assertEquals(Double.NaN, RRTStar7.tLimit(0, 0, 1, 0, 2.5), 0.001);

        // ??
        assertEquals(Double.NaN, RRTStar7.tLimit(7.55, 2.1818, 6.75, 0, 2.5), 0.001);
    }

    @Test
    void testTMirror() {
        // same cases as above.

        // should not exist
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 1, 1, 1, 2), 0.001);

        // (0,1) -> (0.5,1) yields just one intersection
        // so tLimit and tMirror are the same:
        assertEquals(1.000, RRTStar7.tMirror(0, 1, 0.5, 1, 2), 0.001);

        // two intersections, recall tLimit was 0.292
        assertEquals(1.707, RRTStar7.tMirror(0, 1, 0.25, 1, 2), 0.001);

        // two intersections, recall that tLimit was 0.585
        assertEquals(3.414, RRTStar7.tMirror(0, 2, 1, 2, 2), 0.001);

        // (0,0) -> (1,0)
        assertEquals(Double.NaN, RRTStar7.tMirror(0, 0, 1, 0, 2.5), 0.001);

        assertEquals(Double.NaN, RRTStar7.tMirror(7.55, 2.1818, 6.75, 0, 2.5), 0.001);

    }

    @Test
    void quadraticTest() {
        assertEquals(List.of(0.0), Util.quadratic(1, 0, 0));
        assertEquals(List.of(1.0, -1.0), Util.quadratic(1, 0, -1));
        // https://en.wikipedia.org/wiki/Quadratic_formula
        assertEquals(List.of(4.0, 1.0), Util.quadratic(0.5, -2.5, 2));
        assertEquals(List.of(-0.0), Util.quadratic(0, 1, 0));

    }
}
