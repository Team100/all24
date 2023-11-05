package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Test combination of estimator and controller
 */
class EstimatorAndControlTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    // for this test, zero noise
    WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0, 0);
    MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
    final DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);
    final ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);
    final PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
    final LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
    // angle (rad), velocity (rad/s)
    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
    // output (volts)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0);
    GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, kDt);
    Matrix<N1, N2> K = gc.getK();
    final FeedbackControl<N2, N1, N2> controller = new FeedbackControl<>(system, K);

    private RandomVector<N2> updateAndCheck(
            RandomVector<N2> xhat,
            Matrix<N1, N1> u,
            double y,
            double x0,
            double x1) {
        xhat = predictor.predictWithNoise(xhat, u, kDt);
        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(y));
        xhat = pooling.fuse(x, xhat);
        assertEquals(x0, xhat.x.get(0, 0), kDelta);
        assertEquals(x1, xhat.x.get(1, 0), kDelta);
        return xhat;
    }

    private Matrix<N1, N1> controlAndCheck(
            RandomVector<N2> xhat,
            Matrix<N2, N1> setpoint,
            double u0) {
        Matrix<N1, N1> u = controller.calculate(xhat, setpoint);
        assertEquals(u0, u.get(0, 0), kDelta);
        return u;
    }

    @Test
    void testNearZero() {
        // positive setpoint, delta +0.02, push positive

        // initially, state estimate: at zero, motionless
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(0, 0), p);
        assertEquals(0, xhat.x.get(0, 0));
        assertEquals(0, xhat.x.get(1, 0));

        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);

        Matrix<N1, N1> u = controlAndCheck(xhat, setpoint, 11.455);

        xhat = updateAndCheck(xhat, u, 0.002, 0.002, 0.229);
        u = controlAndCheck(xhat, setpoint, 0.151);

        xhat = updateAndCheck(xhat, u, 0.006, 0.006, 0.232);
        u = controlAndCheck(xhat, setpoint, -2.44);

        xhat = updateAndCheck(xhat, u, 0.01, 0.01, 0.184);
        u = controlAndCheck(xhat, setpoint, -2.557);

        xhat = updateAndCheck(xhat, u, 0.013, 0.013, 0.133);
        u = controlAndCheck(xhat, setpoint, -2.015);

        xhat = updateAndCheck(xhat, u, 0.015, 0.015, 0.092);
        u = controlAndCheck(xhat, setpoint, -1.419);

        xhat = updateAndCheck(xhat, u, 0.017, 0.017, 0.063);
        u = controlAndCheck(xhat, setpoint, -1.065);

        xhat = updateAndCheck(xhat, u, 0.018, 0.018, 0.043);
        u = controlAndCheck(xhat, setpoint, -0.726);

        xhat = updateAndCheck(xhat, u, 0.019, 0.019, 0.028);
        u = controlAndCheck(xhat, setpoint, -0.532);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.017);
        u = controlAndCheck(xhat, setpoint, -0.442);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.008);
        u = controlAndCheck(xhat, setpoint, -0.243);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.003);
        u = controlAndCheck(xhat, setpoint, -0.109);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.001);
        u = controlAndCheck(xhat, setpoint, -0.042);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0.0001);
        u = controlAndCheck(xhat, setpoint, -0.013);

        xhat = updateAndCheck(xhat, u, 0.02, 0.02, 0);
        u = controlAndCheck(xhat, setpoint, -0.003);
    }

    @Test
    void testNearPiWithoutWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is pi - 0.03
        // so delta is +0.02, should push positive

        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(Math.PI - 0.03, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(Math.PI - 0.03));
        xhat = pooling.fuse(x, xhat);

        assertEquals(3.112, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(xhat, setpoint, 11.455);

        xhat = updateAndCheck(xhat, u, 3.114, 3.114, 0.229);
        u = controlAndCheck(xhat, setpoint, -0.050);

        xhat = updateAndCheck(xhat, u, 3.118, 3.118, 0.228);
        u = controlAndCheck(xhat, setpoint, -2.509);

        xhat = updateAndCheck(xhat, u, 3.122, 3.122, 0.178);
        u = controlAndCheck(xhat, setpoint, -2.538);

        xhat = updateAndCheck(xhat, u, 3.125, 3.125, 0.128);
        u = controlAndCheck(xhat, setpoint, -1.982);

        xhat = updateAndCheck(xhat, u, 3.128, 3.128, 0.088);
        u = controlAndCheck(xhat, setpoint, -1.563);

        xhat = updateAndCheck(xhat, u, 3.130, 3.130, 0.056);
        u = controlAndCheck(xhat, setpoint, -1.169);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.033);
        u = controlAndCheck(xhat, setpoint, -0.780);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.017);
        u = controlAndCheck(xhat, setpoint, -0.390);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.009);
        u = controlAndCheck(xhat, setpoint, -0.164);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.007);
        u = controlAndCheck(xhat, setpoint, -0.059);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.005);
        u = controlAndCheck(xhat, setpoint, -0.017);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.004);
        u = controlAndCheck(xhat, setpoint, -0.003);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.004);
        u = controlAndCheck(xhat, setpoint, 0.0002);

        xhat = updateAndCheck(xhat, u, 3.131, 3.131, 0.004);
        u = controlAndCheck(xhat, setpoint, 0.0007);
    }

    @Test
    void testNearPiWithWrapping() {
        // example near PI
        // goal is pi-0.01,
        // initial is -pi + 0.01
        // so delta is -0.02, should push negative across the boundary

        // starting point is the only difference
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);

        // initially, state estimate: at zero, motionless
        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // starting point is the only difference

        RandomVector<N2> x = pointEstimator.stateForMeasurementWithZeroU(system.position(-1.0 * Math.PI + 0.01));
        xhat = pooling.fuse(x, xhat);

        assertEquals(-3.132, xhat.x.get(0, 0), kDelta);
        assertEquals(0, xhat.x.get(1, 0), kDelta);

        // try to move +0.02
        Matrix<N2, N1> setpoint = Matrix.mat(Nat.N2(), Nat.N1()).fill(Math.PI - 0.01, 0);
        assertEquals(3.132, setpoint.get(0, 0), kDelta);

        Matrix<N1, N1> u;
        u = controlAndCheck(xhat, setpoint, -11.455);

        xhat = updateAndCheck(xhat, u, -3.133, -3.133, -0.229);
        u = controlAndCheck(xhat, setpoint, -0.251);

        xhat = updateAndCheck(xhat, u, -3.138, -3.138, -0.235);
        u = controlAndCheck(xhat, setpoint, 2.613);

        xhat = updateAndCheck(xhat, u, 3.141, 3.141, -0.182);
        u = controlAndCheck(xhat, setpoint, 2.678);

        xhat = updateAndCheck(xhat, u, 3.138, 3.138, -0.128);
        u = controlAndCheck(xhat, setpoint, 2.061);

        xhat = updateAndCheck(xhat, u, 3.135, 3.135, -0.087);
        u = controlAndCheck(xhat, setpoint, 1.598);

        xhat = updateAndCheck(xhat, u, 3.134, 3.134, -0.055);
        u = controlAndCheck(xhat, setpoint, 1.016);

        xhat = updateAndCheck(xhat, u, 3.133, 3.133, -0.035);
        u = controlAndCheck(xhat, setpoint, 0.661);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.021);
        u = controlAndCheck(xhat, setpoint, 0.491);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.012);
        u = controlAndCheck(xhat, setpoint, 0.259);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.007);
        u = controlAndCheck(xhat, setpoint, 0.113);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.005);
        u = controlAndCheck(xhat, setpoint, 0.042);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.004);
        u = controlAndCheck(xhat, setpoint, 0.013);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.003);
        u = controlAndCheck(xhat, setpoint, 0.003);

        xhat = updateAndCheck(xhat, u, 3.132, 3.132, -0.003);
        u = controlAndCheck(xhat, setpoint, 0.0001);

        // after all that, what's the accumulated variance?
        // position variance is very tiny since we've given it the same measurement over and over.
        // velocity variance is just the prior since we never corrected velocity
        assertArrayEquals(new double[] { 0.000028, 0, 0, 0.100156 }, xhat.Kxx.getData(), 0.000001);
    }
}
