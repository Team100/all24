package org.team100.lib.system;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Represents a plant with nonlinear dynamics.
 * 
 * xdot = f(x,u) + w
 * y = h(x,u) + v
 * 
 * where
 * 
 * x: system state
 * u: control input
 * w: noise process
 * y: measurement output
 * v: measurement noise
 */
public interface NonlinearPlant<States extends Num, Inputs extends Num, Outputs extends Num> {
    /** State evolution */
    RandomVector<States> f(RandomVector<States> x, Matrix<Inputs, N1> u);

    /** Inverse of f with respect to u, for feedforward. */
    Matrix<Inputs, N1> finvWrtU(RandomVector<States> x, RandomVector<States> xdot);

    /**
     * Inverse of f with respect to x, for trending measurement. It's not that
     * important to perfectly invert f; if you get the velocity term, that's
     * probably enough. Use wide variances for unknowns.
     * 
     * This partial-inverse is admittedly a strange thing to do, but I don't really
     * want to take it out, because it illustrates the variance issue with
     * differencing for velocity estimation. If you're not using the TrendEstimator
     * then you don't need to implement this method.
     */
    default RandomVector<States> finvWrtX(RandomVector<States> xdot, Matrix<Inputs, N1> u) {
        throw new UnsupportedOperationException();
    }

    /**
     * Measurement from state. Use wide variances for unknowns.
     * maybe remove u, since it really is never needed.
     * 
     * Usually h is identity, i.e. we can observe the state perfectly.
     */
    RandomVector<Outputs> h(RandomVector<States> x, Matrix<Inputs, N1> u);

    /**
     * Inverse of h with respect to x.
     * 
     * Usually hinv is identity, which indicates that there is no measurement noise,
     * which is wrong of course. TODO: add measurement noise.
     */
    RandomVector<States> hinv(RandomVector<Outputs> y, Matrix<Inputs, N1> u);

    /**
     * "Process noise" aka disturbance.
     */
    WhiteNoiseVector<States> w();

    /** Control limit */
    Matrix<Inputs, N1> limit(Matrix<Inputs, N1> u);

    /** Make a state of the correct type. */
    RandomVector<States> make(Matrix<States, N1> x, Variance<States> Kxx);

    Nat<States> states();

    Nat<Inputs> inputs();

    Nat<Outputs> outputs();
}
