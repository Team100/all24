package org.team100.lib.controller;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Full state controller using constant gain.
 */
public class FeedbackControl<States extends Num, Inputs extends Num, Outputs extends Num> {
    private static final boolean debug = false;
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final Matrix<Inputs, States> m_K;

    public FeedbackControl(
            NonlinearPlant<States, Inputs, Outputs> plant,
            Matrix<Inputs, States> K) {
        m_plant = plant;
        m_K = K;
    }

    /**
     * Returns control output, K(r-x), using constant K.
     * 
     * Output is not aware of actuator limits; clamp the output yourself.
     * 
     * @param x the actual state, xhat from the estimator
     * @param r the desired reference state from the trajectory
     * @return the controller u value. if you want to use this later, e.g. for
     *         correction, you need to remember it.
     */
    public Matrix<Inputs, N1> calculate(RandomVector<States> x, Matrix<States, N1> r) {
        if (debug) System.out.println("x: " + x.x.get(0,0));
        if (debug) System.out.println("r: " + r.get(0,0));
        RandomVector<States> rv = x.make(r, Variance.zero(m_plant.states()));
        if (debug) System.out.println("K: " + m_K);
        Matrix<States, N1> residual = rv.minus(x).x;
        if (debug) System.out.println("residual: " + residual);
        return m_K.times(residual);
    }

    public Matrix<Inputs, N1> calculate(Matrix<States, N1> error) {
        return m_K.times(error);
    }
}