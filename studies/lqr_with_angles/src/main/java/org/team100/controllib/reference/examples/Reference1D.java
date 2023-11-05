package org.team100.controllib.reference.examples;

import org.team100.controllib.reference.Reference;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class Reference1D implements Reference<N2> {

    public double position(double timeSec) {
        return 0;
    }

    public double velocity(double timeSec) {
        return 0;
    }

    public double acceleration(double timeSec) {
        return 0;
    }

    /**
     * desired future state
     */
    public Matrix<N2, N1> getR(double futureTimeSec) {
        double referencePosition = position(futureTimeSec);
        double referenceVelocity = velocity(futureTimeSec);
        return VecBuilder.fill(referencePosition, referenceVelocity);
    }

    /**
     * desired future derivative
     */
    public Matrix<N2, N1> getRDot(double futureTimeSec) {
        double referenceVelocity = velocity(futureTimeSec);
        double referenceAcceleration = acceleration(futureTimeSec);
        return VecBuilder.fill(referenceVelocity, referenceAcceleration);
    }
}
