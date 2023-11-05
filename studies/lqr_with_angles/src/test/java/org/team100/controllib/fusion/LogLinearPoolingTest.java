package org.team100.controllib.fusion;

import static org.junit.jupiter.api.Assertions.assertThrows;

import org.ejml.data.SingularMatrixException;
import org.junit.jupiter.api.Test;
import org.team100.controllib.math.RandomVector;

import edu.wpi.first.math.numbers.N1;

public class LogLinearPoolingTest extends PoolingTest {
    private static final Pooling<N1> p = new DemocraticLogLinearPooling<N1>();

    @Test
    public void testUnanimity() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        assert1(cV, 0, 1);
    }

    @Test
    public void testDifferentMeans() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        assert1(cV, 0.5, 1);
    }

    @Test
    public void testDifferentVariance() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is the same
        // aggregate variance leans towards smaller variance
        assert1(cV, 0.0, 1.333);
    }

    @Test
    public void testDifferent() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate leans towards smaller variance
        assert1(cV, 0.333, 1.333);
    }

    @Test
    public void testZeroVariance() {
        RandomVector<N1> aV = v1(0, 0);
        RandomVector<N1> bV = v1(1, 1);
        // zero variance does not work for geometric mean
        assertThrows(SingularMatrixException.class, () -> p.fuse(aV, bV));
    }

    @Test
    public void testAlmostZero() {
        RandomVector<N1> aV = v1(0, 0.0000001);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // the near-zero variance forces the fusion to (nearly) its estimate
        assert1(cV, 0, 0);
    }

}
