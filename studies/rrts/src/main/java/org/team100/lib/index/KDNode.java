package org.team100.lib.index;

import org.team100.lib.space.Point;

import edu.wpi.first.math.Num;

public final class KDNode<V extends Point<? extends Num>> {
    private final V value;
    private KDNode<V> a;

    private KDNode<V> b;

    public KDNode(V v) {
        if (v == null)
            throw new IllegalArgumentException("null value");
        value = v;
    }

    void setA(KDNode<V> n) {
        a = n;
    }

    void setB(KDNode<V> n) {
        b = n;
    }

    public KDNode<V> getA() {
        return a;
    }

    public KDNode<V> getB() {
        return b;
    }

    public V getValue() {
        return value;
    }

    @Override
    public String toString() {
        return "KDNode [value=" + value + ", a=" + a + ", b=" + b + "]";
    }
}