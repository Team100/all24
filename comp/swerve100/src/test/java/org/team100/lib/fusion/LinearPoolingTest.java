package org.team100.lib.fusion;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class LinearPoolingTest extends PoolingTestUtil {
    private static final double kDelta = 0.001;
    
    private static final LinearPooling<N1> p1 = new LinearPooling<N1>() {

        @Override
        public RandomVector<N1> fuse(RandomVector<N1> a, RandomVector<N1> b) {
            return null;
        }
    };
    private static final LinearPooling<N2> p2 = new LinearPooling<N2>() {

        @Override
        public RandomVector<N2> fuse(RandomVector<N2> a, RandomVector<N2> b) {
            return null;
        }
    };

    @Test
    void testDontKnow1() {
        // fuse an unknown with a known, you should just get the known back.
        RandomVector<N1> aV = v1(0, 1);
        Matrix<N1,N1> pa = VecBuilder.fill(1);
        RandomVector<N1> bV = v1(1, 1e9);
        Matrix<N1,N1> pb = VecBuilder.fill(0);
        RandomVector<N1> cV = p1.fuse(aV, pa, bV, pb);
        assert1(cV, 0, 1);
    }

    @Test
    void testDontKnow2() {
        // fuse an unknown with a known, you should just get the known back.
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1e9);
        RandomVector<N2> bV = v2(1, 1, 1e9, 0, 0, 1);

        Matrix<N2, N2> pa = new Matrix<>(Nat.N2(), Nat.N2());
        pa.set(0, 0, 1);

        Matrix<N2, N2> pb = new Matrix<>(Nat.N2(), Nat.N2());
        pb.set(1, 1, 1);

        assertArrayEquals(new double[]{1,0,0,0},pa.getData(),kDelta);
        assertArrayEquals(new double[]{0,0,0,1},pb.getData(),kDelta); 

        // a is authoritative for x1
        // b is authoritative for x2
        // so c should be [0 1]
        // and variances should be [1 0 0 1]
        RandomVector<N2> cV = p2.fuse(aV, pa, bV, pb);
        assert2(cV, 0, 1, 1, 0, 0, 1);
    }

    @Test
    void testDispersion2() {
        RandomVector<N2> aV = v2(0, 0, 1, 0, 0, 1e9);
        RandomVector<N2> bV = v2(1, 1, 1e9, 0, 0, 1);

        Matrix<N2, N2> pa = new Matrix<>(Nat.N2(), Nat.N2());
        pa.set(0, 0, 1);

        Matrix<N2, N2> pb = new Matrix<>(Nat.N2(), Nat.N2());
        pb.set(1, 1, 1);

        assertArrayEquals(new double[]{1,0,0,0},pa.getData(),kDelta);
        assertArrayEquals(new double[]{0,0,0,1},pb.getData(),kDelta); 

        // a is authoritative for x1
        // b is authoritative for x2
        // so c should be [0 1]
        // and variances should be [1 0 0 1]
        Variance<N2> v = p2.dispersionCovariance(aV, pa, bV, pb);
        assertArrayEquals(new double[]{0,0,0,0},v.getValue().getData(), kDelta);
    }

    @Test
    void testCombinePrimitive() {
        RandomVector<N1> aV = v1(0, 1);
        RandomVector<N1> bV = v1(1, 1e6);
        double pa = 0.999999;
        double pb = 0.000001;
        RandomVector<N1> paaV = aV.times(pa);
        assertArrayEquals(new double[]{0},paaV.x.getData(),kDelta);
        assertArrayEquals(new double[]{1},paaV.Kxx.getData(),kDelta);
        RandomVector<N1> pbbV = bV.times(pb);
        assertArrayEquals(new double[]{0},pbbV.x.getData(),kDelta);
        assertArrayEquals(new double[]{1e-6},pbbV.Kxx.getData(),kDelta);
        RandomVector<N1> cc = paaV.plus(pbbV);
        assertArrayEquals(new double[]{0},cc.x.getData(),kDelta);
        assertArrayEquals(new double[]{1},cc.Kxx.getData(),kDelta);


    }
}
