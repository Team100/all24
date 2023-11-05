package org.team100.lib.index;

public class KDNearNode<V> {
    public final double _dist;
    public final V _nearest;

    public KDNearNode(double dist, V nearest) {
        _dist = dist;
        _nearest = nearest;
    }

    @Override
    public String toString() {
        return "KDNearNode [_dist=" + _dist + ", _nearest=" + _nearest + "]";
    }
}
