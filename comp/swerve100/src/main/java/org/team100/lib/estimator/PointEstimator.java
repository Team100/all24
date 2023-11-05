package org.team100.lib.estimator;

import org.team100.lib.math.RandomVector;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** State from point measurements. */
public class PointEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final Matrix<Inputs, N1> m_uZero;

    public PointEstimator(NonlinearPlant<States, Inputs, Outputs> plant) {
        m_plant = plant;
        m_uZero = new Matrix<>(plant.inputs(), Nat.N1());
    }

    /** In practice, h is u-invariant */
    public RandomVector<States> stateForMeasurementWithZeroU(RandomVector<Outputs> y) {
        return stateForMeasurement(m_uZero, y);
    }

    /**
     * Use the inverse h function to get the state corresponding to the measurement.
     */
    private RandomVector<States> stateForMeasurement(Matrix<Inputs, N1> u, RandomVector<Outputs> y) {
        return m_plant.hinv(y, u);
    }
}
