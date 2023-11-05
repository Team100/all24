package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class InversionFeedforwardTest {
    private static final double kDelta = 0.0001;
    private static final double kDt = 0.02;

    @Test
    void testDoubleIntegrator() {
        // the model is trivial
        // the r and nextR v differ by 1
        // so u should be 1/dt or 50.
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D(w, v);
        InversionFeedforward<N2, N1, N2> feedforward = new InversionFeedforward<>(plant);

        // position does not matter here.
        Matrix<N2, N1> r = VecBuilder.fill(0, 2);
        // for now, a huge step function.
        Matrix<N2, N1> rDot = VecBuilder.fill(1.0 / kDt, 1.0 / kDt);
        Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(r, rDot);
        assertEquals(50, uff.get(0, 0), kDelta);
    }

    @Test
    void testPendulum() {
        // pendulum model vdot includes position dependence
        // the r and nextR differ by 1
        // so u should be 1/dt or 50.

        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D(w, v);
        InversionFeedforward<N2, N1, N2> feedforward = new InversionFeedforward<>(plant);

        {
            // r position 0 means maximum gravity so u = 1
            Matrix<N2, N1> r = VecBuilder.fill(0, 0);
            // Matrix<N2, N1> nextR = VecBuilder.fill(0, 0);
            Matrix<N2, N1> rDot = VecBuilder.fill(0, 0);
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(r, rDot);
            assertEquals(1, uff.get(0, 0), kDelta);
        }
        {
            // r position pi/2 means some gravity so u = sqrt 2
            Matrix<N2, N1> r = VecBuilder.fill(Math.PI / 4, 0);
            Matrix<N2, N1> rDot = VecBuilder.fill(0, 0);
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(r, rDot);
            assertEquals(0.7071, uff.get(0, 0), kDelta);
        }
        {
            // r position pi/2 means no gravity so u = 0
            Matrix<N2, N1> r = VecBuilder.fill(Math.PI / 2, 0);
            Matrix<N2, N1> rDot = VecBuilder.fill(0, 0);
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(r, rDot);
            assertEquals(0, uff.get(0, 0), kDelta);
        }
        {
            // it's easy to construct nonsensical trajectories
            // this one speeds up without moving, so the ff obeys the
            // speeding-up part.
            Matrix<N2, N1> r = VecBuilder.fill(Math.PI / 2, 0);
            // for now, a huge step function
            Matrix<N2, N1> rDot = VecBuilder.fill(0, 1.0 / kDt);
            Matrix<N1, N1> uff = feedforward.calculateWithRAndRDot(r, rDot);
            assertEquals(50, uff.get(0, 0), kDelta);
        }
    }

}
