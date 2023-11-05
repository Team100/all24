package org.team100.lib.reference;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/** Timed reference source */
public interface Reference<States extends Num> {
    /**
     * @return the reference for the specified time
     */
    Matrix<States, N1> getR(double tSec);

    /**
     * The derivative of the trajectory is used directly by feedforward.
     * 
     * @return the first derivative of the trajectory at the specified time
     */
    Matrix<States, N1> getRDot(double tSec);

}
