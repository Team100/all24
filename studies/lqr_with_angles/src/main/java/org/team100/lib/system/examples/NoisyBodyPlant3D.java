package org.team100.lib.system.examples;

import org.team100.controllib.math.MeasurementUncertainty;
import org.team100.controllib.math.RandomVector;
import org.team100.controllib.math.Variance;
import org.team100.controllib.math.WhiteNoiseVector;
import org.team100.controllib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;

public abstract class NoisyBodyPlant3D implements NonlinearPlant<N6, N3, N6> {
    private final WhiteNoiseVector<N6> m_w;
    private final MeasurementUncertainty<N6> m_v;

    public NoisyBodyPlant3D(WhiteNoiseVector<N6> w, MeasurementUncertainty<N6> v) {
        m_w = w;
        m_v = v;
    }

    RandomVector<N6> position(double pX, double pY, double pTheta) {
        Matrix<N6, N1> yx = new Matrix<>(Nat.N6(), Nat.N1());
        yx.set(0, 0, pX);
        yx.set(1, 0, pY);
        yx.set(2, 0, pTheta);
        Variance<N6> yP = m_v.Kxx.copy();
        yP.dontknow(3);
        yP.dontknow(4);
        yP.dontknow(5);
        return make(yx, yP);
    }

    RandomVector<N6> velocity(double vX, double vY, double vTheta) {
        Matrix<N6, N1> yx = new Matrix<>(Nat.N6(), Nat.N1());
        yx.set(3, 0, vX);
        yx.set(4, 0, vY);
        yx.set(5, 0, vTheta);
        Variance<N6> yP = m_v.Kxx.copy();
        yP.dontknow(0);
        yP.dontknow(1);
        yP.dontknow(2);
        return make(yx, yP);
    }

    @Override
    public RandomVector<N6> h(RandomVector<N6> x, Matrix<N3, N1> u) {
        return x;
    }

    @Override
    public RandomVector<N6> hinv(RandomVector<N6> y, Matrix<N3, N1> u) {
        return y;
    }

    @Override
    public WhiteNoiseVector<N6> w() {
        return m_w;
    }

    public MeasurementUncertainty<N6> v() {
        return m_v;
    }

    @Override
    public Matrix<N3, N1> limit(Matrix<N3, N1> u) {
        return StateSpaceUtil.desaturateInputVector(u, 12.0);
    }

    @Override
    public Nat<N6> states() {
        return Nat.N6();
    }

    @Override
    public Nat<N3> inputs() {
        return Nat.N3();
    }

    @Override
    public Nat<N6> outputs() {
        return Nat.N6();
    }
}