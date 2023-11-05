package org.team100.lib.graph;

import edu.wpi.first.math.Num;

/**
 * Near nodes can be sorted by their goodness, so we try
 * to link to the best, i.e. shortest-total-distance, first.
 */
public class NearNode<States extends Num> implements Comparable<NearNode<States>> {
    public final Node<States> node;
    public final double linkDist;
    private final double _pathDist;

    public NearNode(Node<States> node, double linkDist) {
        this.node = node;
        this.linkDist = linkDist;
        if (node.getIncoming() == null) {
            this._pathDist = linkDist;
        } else {
            this._pathDist = node.getIncoming().get_pathDist() + linkDist;
        }
    }

    @Override
    public int compareTo(NearNode<States> o) {
        return Double.compare(_pathDist, o._pathDist);
    }
}
