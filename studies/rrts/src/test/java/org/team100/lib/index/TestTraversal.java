package org.team100.lib.index;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.graph.Node;
import org.team100.lib.space.Point;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class TestTraversal {

    KDModel<N2> myModel = new KDModel<>() {
        @Override
        public Matrix<N2, N1> getMin() {
            return new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0, 0 });
        }

        @Override
        public Matrix<N2, N1> getMax() {
            return new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 1, 1 });
        }

        @Override
        public double dist(Matrix<N2, N1> start, Matrix<N2, N1> end) {
            throw new UnsupportedOperationException("Unimplemented method 'dist'");
        }

        @Override
        public Matrix<N2, N1> steer(KDNearNode<Node<N2>> x_nearest, Matrix<N2, N1> newConfig) {
            throw new UnsupportedOperationException("Unimplemented method 'steer'");
        }

        @Override
        public void setStepNo(int stepNo) {
        }

        @Override
        public void setRadius(double radius) {
        }

    };

    static class StringPoint implements Point<N2> {
        private final String v;
        private final Matrix<N2, N1> _config;

        public StringPoint(String v, Matrix<N2, N1> _config) {
            this.v = v;
            this._config = _config;
        }

        @Override
        public Matrix<N2, N1> getState() {
            return _config;
        }

        public String get_v() {
            return v;

        }
    }

    @Test
    void testInsert() {
        KDNode<StringPoint> n = new KDNode<>(
                new StringPoint("n", new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0, 0 })));
        KDTree.insert(myModel, n, new StringPoint("one", new Matrix<>(Nat.N2(), Nat.N1(), new double[] { 0.5, 0 })));
        List<StringPoint> s = KDTree.values(n);
        assertEquals(2, s.size());
        KDNode<StringPoint> a = n.getA();
        KDNode<StringPoint> b = n.getB();
        assertNull(a);
        assertEquals("one", b.getValue().get_v());

    }

}
