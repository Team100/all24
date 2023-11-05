package org.team100.lib.system.examples;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public abstract class Rotary1D extends NoisyLimitedPlant1D {
    public Rotary1D(WhiteNoiseVector<N2> w, MeasurementUncertainty<N2> v) {
		super(w, v);
	}

	@Override
    public RandomVector<N2> make(Matrix<N2, N1> x, Variance<N2> P) {
        return new AngularRandomVector<>(x,P);
    }
}
