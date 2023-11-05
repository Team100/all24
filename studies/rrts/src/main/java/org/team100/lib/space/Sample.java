package org.team100.lib.space;

import java.util.Random;

import org.team100.lib.index.KDModel;
import org.team100.lib.random.MersenneTwister;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Supplies samples drawn from a uniform distribution across the model bounds.
 */
public class Sample<States extends Num> {
    private final KDModel<States> _kdModel;
    private final Matrix<States, N1> _sampleMin;
    private final Matrix<States, N1> _sampleMax;
    private final Random _random;

    public Sample(KDModel<States> kdModel, int seed) {
        _kdModel = kdModel;
        _random = new MersenneTwister(seed);
        _sampleMin = _kdModel.getMin();
        _sampleMax = _kdModel.getMax();
    }

    public Sample(KDModel<States> kdModel) {
        this(kdModel, 0);
    }

    public Matrix<States, N1> get() {
        Matrix<States, N1> result = _sampleMax.copy();
        for (int i = 0; i < result.getNumRows(); ++i) {
            double range = _sampleMax.get(i, 0) - _sampleMin.get(i, 0);
            result.set(i, 0, _sampleMin.get(i, 0) + range * _random.nextDouble());
        }
        return result;
    }
}
