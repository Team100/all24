package org.team100.lib.system;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.controller.InversionFeedforward;
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
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Demonstrates angle-wrapping with LinearSystemLoop.
 */
class NonlinearSystemLoopTest {
    private static final double kDelta = 0.001;
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance = VecBuilder.fill(0.01, // angle (rad)
            0.2); // velocity (rad/s)
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0); // output (volts)

    // for this test, zero noise
    WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0, 0);
    // note different uncertainties here
    MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.1, 0.1);
    DoubleIntegratorRotary1D system = new DoubleIntegratorRotary1D(w, v);

    GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, kDt);
    Matrix<N1, N2> K = gc.getK();
    FeedbackControl<N2, N1, N2> controller = new FeedbackControl<>(system, K);

    ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(system);
    PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(system);
    LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
    InversionFeedforward<N2, N1, N2> feedforward = new InversionFeedforward<>(system);
    NonlinearSystemLoop<N2, N1, N2> loop = new NonlinearSystemLoop<>(system, predictor, pointEstimator, pooling,
            controller, feedforward);

    @Test
    void testLoop() {
        // initially, state estimate: at zero, motionless
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(0, 0), p);
        assertArrayEquals(new double[] { 0, 0 }, xhat.x.getData(), kDelta);

        // try to get to 0.02
        Vector<N2> setpoint = VecBuilder.fill(0.02, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill(1, 0);

        {
            // initially, push to get started
            xhat = loop.correct(xhat, system.position(0));
            xhat = loop.correct(xhat, system.velocity(0));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(11.455, totalU.get(0, 0), kDelta);

            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.002, 0.229 }, xhat.x.getData(), kDelta);
        }
        // no more change in setpoint after this.
        rDot = VecBuilder.fill(0, 0);

        {
            xhat = loop.correct(xhat, system.position(0.002));
            xhat = loop.correct(xhat, system.velocity(0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(0.067, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.006, 0.231 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.006));
            xhat = loop.correct(xhat, system.velocity(0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.466, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.01, 0.181 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.010));
            xhat = loop.correct(xhat, system.velocity(0.178));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.517, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.014, 0.130 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.014));
            xhat = loop.correct(xhat, system.velocity(0.126));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-2.076, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.016, 0.087 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.016));
            xhat = loop.correct(xhat, system.velocity(0.085));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-1.474, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.017, 0.057 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.017));
            xhat = loop.correct(xhat, system.velocity(0.056));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.962, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.018, 0.039 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.018));
            xhat = loop.correct(xhat, system.velocity(0.037));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.635, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.019, 0.025 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.019));
            xhat = loop.correct(xhat, system.velocity(0.016));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.401, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.019, 0.016 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.020));
            xhat = loop.correct(xhat, system.velocity(0.009));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.288, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.02, 0.009 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(0.020));
            xhat = loop.correct(xhat, system.velocity(0.005));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-0.180, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 0.02, 0.005 }, xhat.x.getData(), kDelta);
        }
        // accumulated variance is pretty tight
        assertArrayEquals(new double[] { 0.0009, 0, 0, 0.0009 }, xhat.Kxx.getData(), 0.0001);
    }

    @Test
    void testWrapping() {
        // start = -pi+0.01
        Variance<N2> p = Variance.from2StdDev(0.316228, 0.316228);
        // p.set(0, 0, 0.1);
        // p.set(1, 1, 0.1);
        RandomVector<N2> xhat = new AngularRandomVector<>(VecBuilder.fill(-1.0 * Math.PI + 0.01, 0), p);
        assertArrayEquals(new double[] { -3.132, 0 }, xhat.x.getData(), kDelta);

        // goal = pi - 0.01
        Vector<N2> setpoint = VecBuilder.fill(Math.PI - 0.01, 0);
        // for this example we're doing a step function
        // in reality we would use a constrained trajectory
        Vector<N2> rDot = VecBuilder.fill((Math.PI - 0.01) / 0.02, 0);

        {
            xhat = loop.correct(xhat, system.position(-1.0 * Math.PI + 0.01));
            xhat = loop.correct(xhat, system.velocity(0));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-11.455, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { -3.133, -0.229 }, xhat.x.getData(), kDelta);
        }
        rDot = VecBuilder.fill(0, 0);
        {
            xhat = loop.correct(xhat, system.position(-3.133));
            xhat = loop.correct(xhat, system.velocity(-0.166));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(-1.559, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { -3.138, -0.230 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(-3.137));
            xhat = loop.correct(xhat, system.velocity(-0.229));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(2.123, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            // crossed the boundary
            assertArrayEquals(new double[] { 3.141, -0.187 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(-3.141));
            xhat = loop.correct(xhat, system.velocity(-0.191));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(2.578, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.138, -0.136 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(3.138));
            xhat = loop.correct(xhat, system.velocity(-0.139));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(2.222, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.136, -0.093 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(3.136));
            xhat = loop.correct(xhat, system.velocity(-0.095));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(1.590, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.134, -0.061 }, xhat.x.getData(), kDelta);
        }
        {
            xhat = loop.correct(xhat, system.position(3.134));
            xhat = loop.correct(xhat, system.velocity(-0.063));
            Matrix<N1, N1> totalU = loop.calculateTotalU(xhat, setpoint, rDot, kDt);
            assertEquals(1.116, totalU.get(0, 0), kDelta);
            xhat = loop.predictState(xhat, totalU, kDt);
            assertArrayEquals(new double[] { 3.133, -0.039 }, xhat.x.getData(), kDelta);
        }
    }
}
