package org.team100.lib.rrt;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.LinkInterface;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNode;
import org.team100.lib.rrt.example.full_state_arena.FullStateHolonomicArena;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.numbers.N4;

public class TestTrees {
    @Test
    void testSteps() {
        FullStateHolonomicArena arena = new FullStateHolonomicArena();
        Node<N4> initial = new Node<>(arena.initial());
        Node<N4> goal = new Node<>(arena.goal());
        assertNull(initial.getIncoming());
        assertNull(goal.getIncoming());
        assertEquals(0, initial.getOutgoingCount());
        assertEquals(0, goal.getOutgoingCount());

        KDNode<Node<N4>> T_a = new KDNode<>(initial);
        KDNode<Node<N4>> T_b = new KDNode<>(goal);
        // note fixed rand seed so the tests here will be deterministic
        RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena, 0), T_a, T_b);
        solver.setRadius(3);
        // it does a bunch of sampling before it finds a feasible point.
        // whoops not anymore.
        // for (int i = 0; i < 3; ++i) {
            assertEquals(0, solver.step());
        // }
        assertEquals(1, solver.step());
        assertNull(initial.getIncoming());
        assertNull(goal.getIncoming());
        assertEquals(1, initial.getOutgoingCount());
        assertEquals(0, goal.getOutgoingCount());
        LinkInterface<N4> link = initial.getOutgoing().next();
        assertEquals(0.891, link.get_linkDist(), 0.001);
        assertEquals(0.891, link.get_pathDist(), 0.001);

        // initial is (15.5,0,6.75,0)
        // source is initial
        assertArrayEquals(new double[] { 15.5, 0, 6.75, 0 }, link.get_source().getState().getData(), 0.001);

        // x is lower than initial but xdot is positive.
        assertArrayEquals(new double[] { 15.418, -0.699, 6.333, 0.173 }, link.get_target().getState().getData(), 0.001);

        
        assertNull(link.get_source().getIncoming());
        assertEquals(0, link.get_target().getOutgoingCount());

    }

}
