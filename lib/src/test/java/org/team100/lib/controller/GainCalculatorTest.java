package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.NonlinearPlant;
import org.team100.lib.system.examples.DoubleIntegratorRotary1D;
import org.team100.lib.system.examples.FrictionCartesian1D;
import org.team100.lib.system.examples.FrictionRotary1D;
import org.team100.lib.system.examples.Pendulum1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

class GainCalculatorTest {
    static final double kDelta = 0.001;

    /** K dependence on x is weak, not worth recalculating K all the time. */
    @Test
    void testPendulumK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D(w, v);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(plant, stateTolerance, controlTolerance, 0.01);
        Matrix<N1, N2> K = gc.getK();
        assertArrayEquals(new double[] { 818.6, 57.5 }, K.getData(), 0.1);
    }

    @Test
    void testFrictionK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new FrictionCartesian1D(w, v);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(plant, stateTolerance, controlTolerance, 0.01);
        Matrix<N1, N2> K = gc.getK();
        assertArrayEquals(new double[] { 822.7, 56.8 }, K.getData(), 0.1);
    }

    /**
     * verify that we're calculating K the same as LQR does using the jacobian
     */
    @Test
    void testFrictionFK2() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new FrictionRotary1D(w, v);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(plant, stateTolerance, controlTolerance, 0.02);
        Matrix<N1, N2> K = gc.getK();
        assertArrayEquals(new double[] { 578.494, 43.763 }, K.getData(), kDelta);
    }

    /**
     * what does the LQR version do
     */
    @Test
    void testFrictionLQRK() {
        // double integrator
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Nat<N2> outputs = Nat.N2();
        // note the last term here
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, -1);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Matrix<N2, N2> C = Matrix.mat(outputs, states).fill(1, 0, 0, 1);
        Matrix<N2, N1> D = Matrix.mat(outputs, inputs).fill(0, 0);
        LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(plant, stateTolerance,
                controlTolerance, 0.02);
        Matrix<N1, N2> K = lqr.getK();
        // less velocity feedback needed because of damping
        assertArrayEquals(new double[] { 578.494, 43.763 }, K.getData(), kDelta);
    }

    @Test
    void testPendulumK2() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new Pendulum1D(w, v);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(plant, stateTolerance, controlTolerance, 0.02);
        Matrix<N1, N2> K = gc.getK();
        // same as double integrator when gravity is max
        assertArrayEquals(new double[] { 572.773, 44.336 }, K.getData(), kDelta);
    }

    /**
     * verify that we're calculating K the same as LQR does using the jacobian
     */
    @Test
    void testFK() {
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D(w, v);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(plant, stateTolerance, controlTolerance, 0.02);
        Matrix<N1, N2> K = gc.getK();
        assertArrayEquals(new double[] { 572.773, 44.336 }, K.getData(), kDelta);
    }

    /**
     * what does the LQR version do
     */
    @Test
    void testLQRK() {
        // double integrator
        Nat<N2> states = Nat.N2();
        Nat<N1> inputs = Nat.N1();
        Nat<N2> outputs = Nat.N2();
        Matrix<N2, N2> A = Matrix.mat(states, states).fill(0, 1, 0, 0);
        Matrix<N2, N1> B = Matrix.mat(states, inputs).fill(0, 1);
        Matrix<N2, N2> C = Matrix.mat(outputs, states).fill(1, 0, 0, 1);
        Matrix<N2, N1> D = Matrix.mat(outputs, inputs).fill(0, 0);
        LinearSystem<N2, N1, N2> plant = new LinearSystem<>(A, B, C, D);
        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.2);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        LinearQuadraticRegulator<N2, N1, N2> lqr = new LinearQuadraticRegulator<>(plant, stateTolerance,
                controlTolerance, 0.02);
        Matrix<N1, N2> K = lqr.getK();
        assertArrayEquals(new double[] { 572.773, 44.336 }, K.getData(), kDelta);
    }
}
