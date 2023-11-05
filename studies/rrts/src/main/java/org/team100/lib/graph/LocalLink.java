package org.team100.lib.graph;

import edu.wpi.first.math.Num;

/**
 * This link type does not keep any information about path distance,
 * just local link distance; it calculates path distance on the fly every time
 * and need not be updated by rewiring.
 */
public class LocalLink<States extends Num> implements LinkInterface<States> {
    /** nullable for root */
    private final Node<States> _source;
    /** nonnull */
    private final Node<States> _target;
    /** length, i.e. cost, of this edge */
    private final double _linkDist;

    /**
     * Create a new link pointing at the node, linkDist away from parent.
     * 
     * @param source
     * @param target
     * @param linkDist distance to the parent
     */
    public LocalLink(Node<States> source, Node<States> target, double linkDist) {
        if (source == null)
            throw new IllegalArgumentException();
        if (target == null)
            throw new IllegalArgumentException();
        if (linkDist < 0)
            throw new IllegalArgumentException();

        _target = target;
        _linkDist = linkDist;
        _source = source;
    }

    @Override
    public Node<States> get_source() {
        return _source;
    }

    /** nonnull */
    @Override
    public Node<States> get_target() {
        return _target;
    }

    @Override
    public double get_linkDist() {
        return _linkDist;
    }

    @Override
    public void set_PathDist(double d) {
        // _pathDist = d;
    }

    /** Total path length from start to here */
    @Override
    public double get_pathDist() {
        if (_source == null) {
            return _linkDist;
        }

        return _source.getPathDist() + _linkDist;
        // return _pathDist;
    }

    @Override
    public String toString() {
        return "LocalLink [_source=" + _source
                + ", _target=" + _target
                + ", _linkDist=" + _linkDist + "]";
    }
}
