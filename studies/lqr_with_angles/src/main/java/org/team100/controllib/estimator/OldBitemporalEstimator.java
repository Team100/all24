package org.team100.controllib.estimator;

import java.util.Map.Entry;

import org.team100.controllib.fusion.LinearPooling;
import org.team100.controllib.math.RandomVector;
import org.team100.controllib.storage.BitemporalBuffer;
import org.team100.controllib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Keeps past states and measurements.
 * 
 * TODO: if asked to provide prediction across past measurements, divide the
 * prediction into pieces and use the measurements.
 * 
 * TODO: differentiate between predictions and corrections?
 */
public class OldBitemporalEstimator<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final BitemporalBuffer<RandomVector<States>> m_stateBuffer;
    private final ExtrapolatingEstimator<States, Inputs, Outputs> m_predictor;
    private final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    private final LinearPooling<States> m_pooling;

    public OldBitemporalEstimator(
            NonlinearPlant<States, Inputs, Outputs> plant,
            BitemporalBuffer<RandomVector<States>> stateBuffer,
            ExtrapolatingEstimator<States, Inputs, Outputs> predictor,
            PointEstimator<States, Inputs, Outputs> pointEstimator,
            LinearPooling<States> pooling) {
        m_stateBuffer = stateBuffer;
        m_predictor = predictor;
        m_pointEstimator = pointEstimator;
        m_pooling = pooling;
    }

    /**
     * Prediction. Find the most-recent state earlier than the specified
     * valid time, and integrate forward to estimate the state at the valid time.
     * Record the new state and time.
     */
    public RandomVector<States> predict(
            Matrix<Inputs, N1> u,
            long recordTimeUSec,
            double validTimeSec) {
        Entry<Double, Entry<Long, RandomVector<States>>> floor = floor(validTimeSec);
        RandomVector<States> xhat = floor.getValue().getValue();
        RandomVector<States> newstate = m_predictor.predict(xhat, u, validTimeSec - floor.getKey());
        return update(newstate, recordTimeUSec, validTimeSec);
    }

    /**
     * Correction. Find the most-recent state earlier than the specified
     * valid time, and use it as the base for correction to estimate the state at
     * the valid time. Record the new state and time.
     * 
     * TODO: this is wrong, it should be extrapolated prior to the first correction, then extrapolated again.
     */
    public RandomVector<States> correct(
            RandomVector<Outputs> y,
            long recordTimeUSec,
            double validTimeSec) {
        // Entry<Double, Entry<Long, RandomVector<States>>> floor = floor(validTimeSec);
        // RandomVector<States> xhat = floor.getValue().getValue();
        RandomVector<States> xhat = m_stateBuffer.floorValue(validTimeSec);

        // ================== wrong here ================
        RandomVector<States> x = m_pointEstimator.stateForMeasurementWithZeroU(y);
        xhat = m_pooling.fuse(x, xhat);
        //xhat = m_estimator.correct(xhat, y, sensor);
        return update(xhat, recordTimeUSec, validTimeSec);
    }

    /**
     * Find the most-recent state earlier than the specified valid time.
     */
    Entry<Double, Entry<Long, RandomVector<States>>> floor(double validTimeSec) {
        return m_stateBuffer.floor(validTimeSec);
    }

    RandomVector<States> update(RandomVector<States> newState, long recordTimeUSec, double validTimeSec) {
        m_stateBuffer.put(recordTimeUSec, validTimeSec, newState);
        return newState;
    }

}
