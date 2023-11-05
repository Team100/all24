package org.team100.lib.system.examples;

import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public abstract class NoisyLimitedPlant1D implements NonlinearPlant<N2, N1, N2> {
    private final WhiteNoiseVector<N2> m_w;
    private final MeasurementUncertainty<N2> m_v;

    public NoisyLimitedPlant1D(WhiteNoiseVector<N2> w, MeasurementUncertainty<N2> v) {
        m_w = w;
        m_v = v;
    }

    /**
     * Construct a random vector representing a measurement of position, using the
     * measurement uncertainty you gave the constructor.
     */
    public RandomVector<N2> position(double p) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(0, 0, p);
        Variance<N2> yP = m_v.Kxx.copy();
        yP.dontknow(1);
        return make(yx, yP);
    }

    /**
     * Construct a random vector representing a measurement of velocity, using the
     * measurement uncertainty you gave the constructor.
     */
    public RandomVector<N2> velocity(double v) {
        Matrix<N2, N1> yx = new Matrix<>(Nat.N2(), Nat.N1());
        yx.set(1, 0, v);
        Variance<N2> yP = m_v.Kxx.copy();
        yP.dontknow(0);
        return make(yx, yP);
    }

    @Override
    public RandomVector<N2> h(RandomVector<N2> x, Matrix<N1, N1> u) {
        return x;
    }

    @Override
    public RandomVector<N2> hinv(RandomVector<N2> y, Matrix<N1, N1> u) {
        return y;
    }

    @Override
    public WhiteNoiseVector<N2> w() {
        return m_w;
    }

    public MeasurementUncertainty<N2> v() {
        return m_v;
    }

    @Override
    public Matrix<N1, N1> limit(Matrix<N1, N1> u) {
        return StateSpaceUtil.desaturateInputVector(u, 12.0);
    }

    @Override
    public Nat<N2> states() {
        return Nat.N2();
    }

    @Override
    public Nat<N1> inputs() {
        return Nat.N1();
    }

    @Override
    public Nat<N2> outputs() {
        return Nat.N2();
    }
}
