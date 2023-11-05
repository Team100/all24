package org.team100.lib.index;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import org.team100.lib.space.Point;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

public class KDTree {

    /** Returns all the values in the subtree. */
    public static <States extends Num, V extends Point<States>> List<V> values(KDNode<V> root) {
        List<V> list = new ArrayList<>();
        buildList(list, root);
        return list;
    }

    private static <States extends Num, V extends Point<States>> void buildList(List<V> list, KDNode<V> node) {
        if (node == null)
            return;
        list.add(node.getValue());
        buildList(list, node.getA());
        buildList(list, node.getB());
    }

    /** Inserts the value into the KD Tree. */
    public static <States extends Num, V extends Point<States>> void insert(KDModel<States> model, KDNode<V> root,
            V value) {
        Matrix<States, N1> min = model.getMin();
        Matrix<States, N1> max = model.getMax();

        KDNode<V> newNode = new KDNode<>(value);
        KDNode<V> n = root;
        int depth = 0;

        for (;; ++depth) {
            int axis = depth % min.getNumRows();
            double mp = (min.get(axis, 0) + max.get(axis, 0)) / 2;
            double v = value.getState().get(axis, 0);

            if (v < mp) {
                // a-side
                if (n.getA() == null) {
                    n.setA(newNode);
                    break;
                }
                max.set(axis, 0, mp);
                n = n.getA();
            } else {
                // b-side
                if (n.getB() == null) {
                    n.setB(newNode);
                    break;
                }
                min.set(axis, 0, mp);
                n = n.getB();
            }
        }
    }

    /**
     * @param consumer Consumes possible parents for target.
     */
    public static <States extends Num, V extends Point<States>> void near(
            KDModel<States> model,
            KDNode<V> root,
            Matrix<States, N1> target,
            double radius,
            BiConsumer<V, Double> consumer) {
        Matrix<States, N1> min = model.getMin();
        Matrix<States, N1> max = model.getMax();
        KDTree.near(model, min, max, consumer, root, target, radius, 0);
    }

    /**
     * @param consumer Consumes possible parents for target.
     */
    private static <States extends Num, V extends Point<States>> void near(
            KDModel<States> model,
            Matrix<States, N1> min,
            Matrix<States, N1> max,
            BiConsumer<V, Double> consumer,
            KDNode<V> kdNode,
            Matrix<States, N1> target,
            double radius,
            int depth) {
        final double dist = model.dist(kdNode.getValue().getState(), target);
        if (dist < radius) {
            consumer.accept(kdNode.getValue(), dist);
        }
        final int axis = depth % min.getNumRows();
        final double mp = (min.get(axis, 0) + max.get(axis, 0)) / 2;
        final double dm = Math.abs(mp - target.get(axis, 0));

        KDNode<V> a = kdNode.getA();

        if (a != null && (target.get(axis, 0) < mp || dm < radius)) {
            // in or near a-side
            double tmp = max.get(axis, 0);
            max.set(axis, 0, mp);
            near(model, min, max, consumer, a, target, radius, depth + 1);
            max.set(axis, 0, tmp);
        }

        KDNode<V> b = kdNode.getB();

        if (b != null && (mp <= target.get(axis, 0) || dm < radius)) {
            // in or near b-side
            double tmp = min.get(axis, 0);
            min.set(axis, 0, mp);
            near(model, min, max, consumer, b, target, radius, depth + 1);
            min.set(axis, 0, tmp);
        }
    }

    public static <States extends Num, V extends Point<States>> KDNearNode<V> nearest(KDModel<States> model,
            KDNode<V> root, Matrix<States, N1> target) {
        Matrix<States, N1> min = model.getMin();
        Matrix<States, N1> max = model.getMax();
        return KDTree.nearest(new KDNearNode<V>(Double.MAX_VALUE, null), model, root, min, max, target, 0);
    }

    public static <States extends Num, V extends Point<States>> KDNearNode<V> nearest(
            KDNearNode<V> best,
            KDModel<States> model,
            KDNode<V> n,
            Matrix<States, N1> min,
            Matrix<States, N1> max,
            Matrix<States, N1> target,
            int depth) {
        final int axis = depth % min.getNumRows();
        final double d = model.dist(n.getValue().getState(), target);

        if (d < best._dist) {
            best = new KDNearNode<>(d, n.getValue());
        }

        final double mp = (min.get(axis, 0) + max.get(axis, 0)) / 2;

        if (target.get(axis, 0) < mp) {
            // a-side
            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = max.get(axis, 0);
                max.set(axis, 0, mp);
                best = nearest(best, model, a, min, max, target, depth + 1);
                max.set(axis, 0, tmp);
            }

            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = Math.abs(mp - target.get(axis, 0));
                if (tmp < best._dist) {
                    tmp = min.get(axis, 0);
                    min.set(axis, 0, mp);
                    best = nearest(best, model, b, min, max, target, depth + 1);
                    min.set(axis, 0, tmp);
                }
            }
        } else {
            // b-side
            KDNode<V> b = n.getB();
            if (b != null) {
                double tmp = min.get(axis, 0);
                min.set(axis, 0, mp);
                best = nearest(best, model, b, min, max, target, depth + 1);
                min.set(axis, 0, tmp);
            }

            KDNode<V> a = n.getA();
            if (a != null) {
                double tmp = Math.abs(mp - target.get(axis, 0));
                if (tmp < best._dist) {
                    tmp = max.get(axis, 0);
                    max.set(axis, 0, mp);
                    best = nearest(best, model, a, min, max, target, depth + 1);
                    max.set(axis, 0, tmp);
                }
            }
        }
        return best;
    }

    private KDTree() {
    }

}
