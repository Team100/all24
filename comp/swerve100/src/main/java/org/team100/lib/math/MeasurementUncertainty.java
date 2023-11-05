package org.team100.lib.math;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;

/**
 * Represents uncertainty, v in the measurement equation:
 * 
 * y = h(x, t) + v(t)
 * 
 * Represents the beliefs of of the sensors in a plant.
 * 
 * https://en.wikipedia.org/wiki/Measurement_uncertainty
 */
public class MeasurementUncertainty<States extends Num> {
    public final Variance<States> Kxx;

    public MeasurementUncertainty(Variance<States> Kxx) {
        this.Kxx = Kxx;
    }

    public static MeasurementUncertainty<N2> for2(double sigma1, double sigma2) {
        return new MeasurementUncertainty<>(Variance.from2StdDev(sigma1, sigma2));
    }
}
