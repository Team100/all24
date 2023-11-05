package org.team100.lib.system.examples;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional pendulum with gravity. Angle is measured from horizontal.
 * State includes velocity and position, input is acceleration, output is
 * position.
 */
public class Pendulum1D extends Rotary1D {
    public Pendulum1D(WhiteNoiseVector<N2> w, MeasurementUncertainty<N2> v) {
        super(w, v);
    }

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - cos(p)
     * 
     * so vdot itself depends on p but it is still linear in u.
     */
    @Override
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double p = xmat.x.get(0, 0);
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - Math.cos(p);
        Matrix<N2, N1> xdotx = VecBuilder.fill(pdot, vdot);
        Matrix<N2,N2> xdotP = xmat.Kxx.copy().getValue();
        xdotP.fill(0);
        // propagate variance of x through f (u has zero variance)
        double pP = xmat.Kxx.get(0, 0);
        double vP = xmat.Kxx.get(1, 1);
        xdotP.set(0, 0, vP);
        // https://en.wikipedia.org/wiki/Propagation_of_uncertainty
        xdotP.set(1, 1, Math.pow(Math.sin(p), 2) * pP);
        return new RandomVector<>(xdotx, new Variance<>(xdotP));
    }

    @Override
    public Matrix<N1, N1> finvWrtU(RandomVector<N2> x, RandomVector<N2> xdot) {
        double a = xdot.x.get(1, 0);
        double p = x.x.get(0, 0);
        return VecBuilder.fill(a + Math.cos(p));
    }

    /**
     * since the cos part is not invertible i'm leaving it out; this just turns
     * position changes into velocity.
     */
    @Override
    public RandomVector<N2> finvWrtX(RandomVector<N2> xdot, Matrix<N1, N1> u) {
        double pdot = xdot.x.get(0, 0);
        double v = pdot;
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        xx.set(1, 0, v);
        Matrix<N2,N2> xP = new Matrix<>(Nat.N2(),Nat.N2());
        xP.set(0, 0, 1e9); // position: "don't know" variance
        xP.set(1, 1, xdot.Kxx.get(0, 0)); // better P?
        // Full state, return angular.
        return new AngularRandomVector<>(xx, new Variance<>(xP));
    }
}
