package org.team100.lib.rrt;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Node;
import org.team100.lib.index.KDNode;
import org.team100.lib.rrt.example.full_state_arena.FullStateHolonomicArena;
import org.team100.lib.space.Sample;

import edu.wpi.first.math.numbers.N4;

public class OptimizeTest {
    @Test
    void testOptimize() {
        final FullStateHolonomicArena arena = new FullStateHolonomicArena();
        KDNode<Node<N4>> T_a = new KDNode<>(new Node<>(arena.initial()));
        KDNode<Node<N4>> T_b = new KDNode<>(new Node<>(arena.goal()));
        final RRTStar7<FullStateHolonomicArena> solver = new RRTStar7<>(arena, new Sample<>(arena), T_a, T_b);

    }
    
}
