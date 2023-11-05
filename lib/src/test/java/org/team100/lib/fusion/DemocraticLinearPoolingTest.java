package org.team100.lib.fusion;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.numbers.N1;

class DemocraticLinearPoolingTest extends PoolingTestUtil {
    private static final Pooling<N1> p = new DemocraticLinearPooling<N1>();

    @Test
    void testUnanimity() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // fuse with yourself => more sure
        assert1(cV, 0, 0.5);
    }

    @Test
    void testDifferentMeans() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance is 0.5 as above plus 0.25 from dispersion
        assert1(cV, 0.5, 0.75);
    }

    @Test
    void testDifferentVariance() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(0, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is the same
        // variance is is lower because means are the same
        assert1(cV, 0.0, 0.75);
    }

    @Test
    void testDifferent() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 2);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean is right in the middle
        // aggregate variance is a bit bigger, both mean and variance affect it
        assert1(cV, 0.5, 1);
    }

    @Test
     void testZeroVariance() {
        RandomVector<N1> aV = v1(0, 0);
        RandomVector<N1> bV = v1(1, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // aggregate mean in the middle
        // aggregate variance is ... ok i guess?
        assert1(cV, 0.5, 0.5);
    }

    @Test
    void testNotWrapping() {
        // in this example position is an angle
        // but wrapping is not required
        AngularRandomVector<N1> aV = a1(- Math.PI / 4, 1);
        AngularRandomVector<N1> bV = a1(Math.PI / 4 + 0.01, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // so the mean should be around zero
        // and the variance is a bit bigger since the inputs are pi/2 apart
        assert1(cV, 0.005, 1.125);
    }

    @Test
    void testWrapping() {
        // in this example position is an angle
        AngularRandomVector<N1> aV = a1(-3 * Math.PI / 4, 1);
        // a little less so it ends up positive
        AngularRandomVector<N1> bV = a1(3 * Math.PI / 4 - 0.01, 1);
        RandomVector<N1> cV = p.fuse(aV, bV);
        // so the mean should be around pi
        // and the variance is a bit bigger since the inputs are pi/2 apart
        assert1(cV, Math.PI - 0.005, 1.125);
    }
}
