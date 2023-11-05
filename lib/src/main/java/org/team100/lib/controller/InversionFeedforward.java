package org.team100.lib.controller;

import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * General plant inverse feedforward.
 * 
 * For general system dynamics:
 * 
 * dx/dt = f(x,u)
 * 
 * Feedforward is calculated using the provided f inverse:
 * 
 * u = finv(x, dx/dt)
 */
public class InversionFeedforward<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;

    /**
     * @param plant includes f and finv
     */
    public InversionFeedforward(NonlinearPlant<States, Inputs, Outputs> plant) {
        m_plant = plant;
    }

    /**
     * @param r    the desired state
     * @param rdot rate of change of desired state
     * @return feedforward as linearized and inverted control response
     */
    public Matrix<Inputs, N1> calculateWithRAndRDot(Matrix<States, N1> r, Matrix<States, N1> rDot) {
        RandomVector<States> rv = new RandomVector<>(r, Variance.zero(m_plant.states()));
        RandomVector<States> rdotv = new RandomVector<>(rDot, Variance.zero(m_plant.states()));
        return m_plant.finvWrtU(rv, rdotv);
    }
}
