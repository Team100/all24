package org.team100.lib.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.controller.GainCalculator;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.reference.Reference;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorCartesian1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class BitemporalEstimatorControllerTest {
    private static final double dtSec = 0.02;

    /** Tests a very simple case with zero acceleration (so zero required input). */
    @Test
    void testSteadyState() {
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> system = new DoubleIntegratorCartesian1D(wx, vx);

        // initial state is at zero but moving with velocity one
        Variance<N2> px = Variance.from2StdDev(.01, .01);
        RandomVector<N2> initialState = new RandomVector<>(VecBuilder.fill(0, 1), px);

        // zero control output
        Matrix<N1, N1> initialControl = VecBuilder.fill(0);

        // reference is increasing position, constant velocity, zero acceleration.
        Reference<N2> reference = new Reference<>() {
            @Override
            public Matrix<N2, N1> getR(double tSec) {
                return VecBuilder.fill(tSec, 1);
            }

            @Override
            public Matrix<N2, N1> getRDot(double tSec) {
                return VecBuilder.fill(1, 0);
            }
        };

        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(1.0);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, dtSec);

        // i have no idea if these gains are reasonable.
        Matrix<N1, N2> K = gc.getK();
        assertEquals(86, K.get(0, 0), 0.1);
        assertEquals(13.8, K.get(0, 1), 0.1);

        BitemporalEstimatorController<N2, N1, N2> controller = new BitemporalEstimatorController<>(
                system, initialState, initialControl, reference, K);

        // position 1, velocity 1, time 1, should be equal to the reference.
        RandomVector<N2> measurement = new RandomVector<>(VecBuilder.fill(1, 1), px);
        controller.acceptMeasurement(1, 1, measurement);

        // replay should catch the one measurement we added
        int count = controller.replay(1000000);
        assertEquals(1, count);

        // at 2 sec we should be at position 2 with velocity 1
        RandomVector<N2> prediction = controller.predictNow(2);
        assertEquals(2, prediction.x.get(0, 0), 0.001);
        assertEquals(1, prediction.x.get(1, 0), 0.001);

        // since acceleration is zero, feedforward is also zero
        Matrix<N1, N1> uff = controller.calculateFeedforward(3);
        assertEquals(0, uff.get(0, 0), 0.001);

        // at 4 sec, which is 2 seconds more, we're at 4 with velocity 1
        RandomVector<N2> predictWithFF = controller.predictFutureUsingFF(prediction, uff, 2);
        assertEquals(4, predictWithFF.x.get(0, 0));
        assertEquals(1, predictWithFF.x.get(1, 0));

        // since the measurement is on the reference the feedback output is zero
        Matrix<N1, N1> ufb = controller.calculateFeedback(4, predictWithFF);
        assertEquals(0, ufb.get(0, 0));

        controller.recordHistory(4, ufb);
    }

    /**
     * Tests a sinusoidal double-integrator, which means sinusoidal input,
     * velocity, etc.
     */
    @Test
    void testSinusoidal() {
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> system = new DoubleIntegratorCartesian1D(wx, vx);

        // initial state is at zero but moving with velocity one
        Variance<N2> px = Variance.from2StdDev(.01, .01);
        RandomVector<N2> initialState = new RandomVector<>(VecBuilder.fill(0, 1), px);

        // initial state has no output
        Matrix<N1, N1> initialControl = VecBuilder.fill(0);

        // reference position is sin(t), velocity is cos(t), accel is -sin(t).
        Reference<N2> reference = new Reference<>() {
            @Override
            public Matrix<N2, N1> getR(double tSec) {
                return VecBuilder.fill(Math.sin(tSec), Math.cos(tSec));
            }

            @Override
            public Matrix<N2, N1> getRDot(double tSec) {
                return VecBuilder.fill(Math.cos(tSec), -Math.sin(tSec));
            }
        };

        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(1.0);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance, dtSec);

        // i have no idea if these gains are reasonable.
        Matrix<N1, N2> K = gc.getK();
        assertEquals(86, K.get(0, 0), 0.1);
        assertEquals(13.8, K.get(0, 1), 0.1);

        BitemporalEstimatorController<N2, N1, N2> controller = new BitemporalEstimatorController<>(
                system, initialState, initialControl, reference, K);

        double actual_x = 0;
        double actual_v = 1;
        double actual_a = 0;
        double u = 0;
        long recordTime = 0;

        // use a small time step to stay close to the reference
        for (double t = 0; t < 1.57; t += dtSec) {
            recordTime += 1;

            RandomVector<N2> measurement = new RandomVector<>(VecBuilder.fill(actual_x, actual_v), px);
            controller.acceptMeasurement(recordTime, t, measurement);
            controller.replay(recordTime);
            RandomVector<N2> prediction = controller.predictNow(t);
            Matrix<N1, N1> uff = controller.calculateFeedforward(t + dtSec / 2);
            RandomVector<N2> predictWithFF = controller.predictFutureUsingFF(prediction, uff, dtSec);
            Matrix<N1, N1> ufb = controller.calculateFeedback(t + dtSec, predictWithFF);
            u = uff.plus(ufb).get(0, 0);
            actual_a = u;
            actual_v += actual_a * dtSec;
            actual_x += actual_v * dtSec + actual_a * dtSec * dtSec / 2;
            controller.recordHistory(t, uff.plus(ufb));

            // this controller keeps pretty close to the reference position
            assertEquals(actual_x, reference.getR(t).get(0, 0), 0.02);
            // and velocity
            assertEquals(actual_v, reference.getR(t).get(1, 0), 0.01);
            // and acceleration
            assertEquals(actual_a, reference.getRDot(t).get(1, 0), 0.04);
        }
    }
}
