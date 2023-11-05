package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.examples.FrictionCartesian1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class FrictionCartesian1DTest {
    private static final double kDt = 0.02;
    private static final double kDelta = 0.001;

    /**
     * Sanity-check the behavior of "f" in one of the system classes.
     */
    @Test
    void testFictionCartesian1D() {
        // double integrator with velocity-proportional friction.
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        FrictionCartesian1D system_x = new FrictionCartesian1D(wx, vx);

        ExtrapolatingEstimator<N2, N1, N2> estimator = new ExtrapolatingEstimator<>(system_x);

        // starting at zero...
        Variance<N2> px = Variance.from2StdDev(.01, .01);
        RandomVector<N2> xhat_x = new RandomVector<>(VecBuilder.fill(0, 0), px);
        // with output of, say, one:
        Matrix<N1, N1> u = VecBuilder.fill(1);

        xhat_x = estimator.predictWithNoise(xhat_x, u, kDt);

        // the dynamics are xdot = v, vdot = u - v
        // so since v is zero at this time slice then x should still be zero
        assertEquals(0, xhat_x.x.get(0, 0), kDelta);
        // and v should be 0.02.
        assertEquals(0.02, xhat_x.x.get(1, 0), kDelta);

        // in steady state, u = v, so integrate for awhile to get there:
        for (int i = 0; i < 10000; ++i) {
            xhat_x = estimator.predictWithNoise(xhat_x, u, kDt);
        }
        // x will be continuously increasing so don't bother testing it.
        // assertEquals(0, xhat_x.x.get(0,0), kDelta);
        // the steady-state v is when v = u = 1:
        assertEquals(1, xhat_x.x.get(1, 0), kDelta);
    }
}
