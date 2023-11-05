package org.team100.lib.graph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import org.team100.lib.space.Point;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** TODO: add u value */
public class Node<States extends Num> implements Point<States> {
    private final Matrix<States, N1> state;

    /** Nullable for root, can be updated. */
    private LinkInterface<States> incoming;

    private Set<LinkInterface<States>> outgoing;

    public Node(Matrix<States, N1> state) {
        this.state = state;
        this.outgoing = new HashSet<>();
    }

    @Override
    public Matrix<States, N1> getState() {
        return state;
    }

    public void setIncoming(LinkInterface<States> link) {
        incoming = link;
    }

    /** Nullable for root (i.e. start). */
    public LinkInterface<States> getIncoming() {
        return incoming;
    }

    public void addOutgoing(LinkInterface<States> link) {
        outgoing.add(link);
    }

    public void removeOutgoing(LinkInterface<States> link) {
        outgoing.remove(link);
    }

    public Iterator<LinkInterface<States>> getOutgoing() {
        // copy to avoid concurrent modification exception
        return new ArrayList<>(outgoing).iterator();
    }

    public int getOutgoingCount() {
        return outgoing.size();
    }

    /** The path distance of the incoming link, if any. */
    public double getPathDist() {
        if (incoming == null)
            return 0;
        return incoming.get_pathDist();
    }

    @Override
    public String toString() {
        return "Node [state=" + state.toString() + "]";
    }
}
