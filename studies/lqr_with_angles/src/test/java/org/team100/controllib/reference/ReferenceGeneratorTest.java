package org.team100.controllib.reference;

import org.junit.jupiter.api.Test;
import org.team100.controllib.estimator.ExtrapolatingEstimator;
import org.team100.controllib.estimator.PointEstimator;
import org.team100.controllib.fusion.LinearPooling;
import org.team100.controllib.fusion.VarianceWeightedLinearPooling;
import org.team100.controllib.math.MeasurementUncertainty;
import org.team100.controllib.math.WhiteNoiseVector;
import org.team100.controllib.system.NonlinearPlant;
import org.team100.controllib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ReferenceGeneratorTest {

    @Test
    public void testSimple() {
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01,0.1);
        NonlinearPlant<N2, N1, N2> plant = new DoubleIntegratorRotary1D(w,v);
        ExtrapolatingEstimator<N2, N1, N2> predictor = new ExtrapolatingEstimator<>(plant);
        PointEstimator<N2, N1, N2> pointEstimator = new PointEstimator<>(plant);
        LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
        ReferenceGenerator rg = new ReferenceGenerator();
    }
}
