package org.team100.lib.planner;

import java.util.List;

import org.team100.lib.graph.Node;
import org.team100.lib.space.Path;
import org.team100.lib.space.SinglePath;

import edu.wpi.first.math.Num;

/**
 * Interface for solvers, can be stepped and listeners can see incremental
 * results.
 */
public interface Solver<States extends Num> {
    /** Used to adjust radius. */
    void setStepNo(int stepNo);

    /** Try to add an edge, return number added. */
    int step();

    /** Return the whole tree. */
    List<Node<States>> getNodesA();
    List<Node<States>> getNodesB();

    /** The best path so far, or null if no path spans the start and end states. */
    Path<States> getBestPath();
    SinglePath<States> getBestSinglePath();
}
