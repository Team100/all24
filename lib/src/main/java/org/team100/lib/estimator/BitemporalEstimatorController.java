package org.team100.lib.estimator;

import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.InversionFeedforward;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.RandomVector;
import org.team100.lib.reference.Reference;
import org.team100.lib.storage.BitemporalBuffer;
import org.team100.lib.storage.EditableHistory;
import org.team100.lib.storage.History;
import org.team100.lib.system.NonlinearPlant;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * Orchestrator for estimation and control.
 * 
 * There are two facets: acceptimg measurements and determining control.
 * 
 * measurements can be accepted at any time in any threads:
 * 
 * acceptMeasurement(systemTime, validTime, measurement)
 *
 * for control, use this sequence:
 * 
 * // apply any new measurements, rewriting state history
 * replay(systemTimeMicrosec);
 * 
 * // integrate to the present instant
 * RandomVector<States> current = predictNow(actualTimeSec);
 * 
 * // use midpoint feedforward as the future control
 * Matrix<Inputs, N1> uff = calculateFeedforward(actualTimeSec + kDt / 2);
 *
 * // integrate to the end of the next period
 * RandomVector<States> predicted = predictFutureUsingFF(current, uff, kDt);
 *
 * // calculate extra control required
 * Matrix<Inputs, N1> ufb = calculateFeedback(actualTimeSec + kDt, predicted);
 *
 * // this is the control to apply
 * Matrix<Inputs, N1> u = ufb.plus(uff);
 *
 * // record if you actually use it
 * record(actualTimeSec, u);
 *
 */
public class BitemporalEstimatorController<States extends Num, Inputs extends Num, Outputs extends Num> {
    private static final boolean kDebug = false;

    public final RandomVector<States> m_initialState;
    public final Matrix<Inputs, N1> m_initialControl;
    public final Reference<States> m_reference;
    // measurements are bitemporal so we can notice late-arriving ones
    public final BitemporalBuffer<RandomVector<Outputs>> m_measurements;
    // we rewrite recent state history as needed.
    public final EditableHistory<RandomVector<States>> m_estimates;
    // control history is immutable.
    public final History<Matrix<Inputs, N1>> m_control_history;
    public final ExtrapolatingEstimator<States, Inputs, Outputs> m_predictor;
    public final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    public final TrendEstimator<States, Inputs, Outputs> m_trendEstimator;
    public final LinearPooling<States> m_pooling;
    public final InversionFeedforward<States, Inputs, Outputs> m_feedforward;
    public final FeedbackControl<States, Inputs, Outputs> m_feedback;

    // the last recordTime we've seen from the buffer
    private long recordTime;

    /**
     * @param system         plant dynamics
     * @param initialState   used by the predictor
     * @param initialControl used by the predictor
     * @param reference      produces trajectory
     * @param K              feedback gain
     */
    public BitemporalEstimatorController(
            NonlinearPlant<States, Inputs, Outputs> system,
            RandomVector<States> initialState,
            Matrix<Inputs, N1> initialControl,
            Reference<States> reference,
            Matrix<Inputs, States> K) {
        m_initialState = initialState;
        m_initialControl = initialControl;
        m_reference = reference;
        m_measurements = new BitemporalBuffer<>(1000);
        m_estimates = new EditableHistory<>(1000);
        m_control_history = new History<>(1000);
        m_predictor = new ExtrapolatingEstimator<>(system);
        m_pointEstimator = new PointEstimator<>(system);
        m_trendEstimator = new TrendEstimator<>(system);
        m_pooling = new VarianceWeightedLinearPooling<>();
        m_feedforward = new InversionFeedforward<>(system);
        m_feedback = new FeedbackControl<>(system, K);
    }

    /**
     * Accepts a measurement about some time in the past. These are collected later
     * by replay. Threadsafe.
     */
    public void acceptMeasurement(long recordTimeUs, double validTimeSec, RandomVector<Outputs> measurement) {
        m_measurements.put(recordTimeUs, validTimeSec, measurement);
    }

    /**
     * Updates the state history with any measurements that are pending.
     * 
     * @return the number of measurements found to replay
     */
    public int replay(long currentSystemTimeMicrosec) {
        double earliestMeasurementSec = m_measurements.earliestValidTimeForRecordsAfter(recordTime);
        recordTime = currentSystemTimeMicrosec;

        // we need to replay all the measurements since then
        NavigableMap<Double, Entry<Long, RandomVector<Outputs>>> todo = m_measurements
                .validTailMap(earliestMeasurementSec);

        // we don't need the old estimates, we're going to redo them all
        m_estimates.trim(earliestMeasurementSec);

        // loop through the measurements to replay, in valid-time order.
        int replayCount = 0;
        if (kDebug)
            System.out.println("replay " + todo.size());
        for (Entry<Double, Entry<Long, RandomVector<Outputs>>> measurementEntry : todo.entrySet()) {
            replayCount += 1;
            // find the time of the measurement
            double measurementTime = measurementEntry.getKey();
            // find the most-recent state earlier than the measurement
            Entry<Double, RandomVector<States>> entry = m_estimates.floor(measurementTime);
            if (entry == null) {
                entry = Map.entry(0.0, m_initialState);
            }
            double stateTimeS = entry.getKey();
            RandomVector<States> priorState = entry.getValue();
            if (kDebug)
                System.out.println("found position " + priorState.x.get(0, 0));

            // this is the control in use at the time of the prior state
            Entry<Double, Matrix<Inputs, N1>> historicalUEntry = m_control_history.floor(stateTimeS);
            if (historicalUEntry == null)
                historicalUEntry = Map.entry(0.0, m_initialControl);
            Matrix<Inputs, N1> historical_u = historicalUEntry.getValue();
            if (kDebug)
                System.out.println("found u " + historical_u);

            // there could be multiple controls between the state and the measurement.
            // so make a new state for each control
            NavigableMap<Double, Matrix<Inputs, N1>> uEntries = m_control_history.validSubMap(stateTimeS,
                    measurementTime);
            for (Entry<Double, Matrix<Inputs, N1>> uEntry : uEntries.entrySet()) {
                // integrate the prior state and the prior u up to the new u
                double endS = uEntry.getKey();
                double integrationSpanS = endS - stateTimeS;
                if (kDebug)
                    System.out
                            .println("tween integrating from " + stateTimeS + " to " + endS + " u " + historical_u);
                if (kDebug)
                    System.out.println("tween prior state " + priorState.x.get(0, 0));
                priorState = m_predictor.predictWithNoise(
                        priorState,
                        historical_u,
                        integrationSpanS);
                if (kDebug)
                    System.out.println("tween estimate position " + priorState.x.get(0, 0));

                historical_u = uEntry.getValue();
                stateTimeS = endS;
            }
            // now the prior state and state time are up to the most-recent change in u
            // and the historical u is also the latest u
            // so integrate the last little bit
            if (kDebug)
                System.out.println(
                        "replay integrating from " + stateTimeS + " to " + measurementTime + " u " + historical_u);
            if (kDebug)
                System.out.println("replay prior state " + priorState.x.get(0, 0));
            double stateToMeasurementS = measurementTime - stateTimeS;
            RandomVector<States> predictedState = m_predictor.predictWithNoise(
                    priorState,
                    historical_u,
                    stateToMeasurementS);
            if (kDebug)
                System.out.println("replay estimate " + predictedState.x.get(0, 0));

            // this is the measurement state
            RandomVector<States> measurementState = m_pointEstimator
                    .stateForMeasurementWithZeroU(measurementEntry.getValue().getValue());
            if (kDebug)
                System.out.println("replay measurement " + measurementState.x.get(0, 0));
            // pool the measurement and the extrapolation
            RandomVector<States> fused = m_pooling.fuse(predictedState, measurementState);
            // record the new estimate
            if (kDebug)
                System.out.println("replay fused " + fused.x.get(0, 0));
            m_estimates.put(measurementTime, fused);
        }
        return replayCount;
    }

    /**
     * Predicts the state for the current instant.
     */
    public RandomVector<States> predictNow(double currentTimeSec) {
        Entry<Double, RandomVector<States>> entry = m_estimates.floor(currentTimeSec);
        if (entry == null)
            entry = Map.entry(0.0, m_initialState);

        double stateTimeS = entry.getKey();
        RandomVector<States> priorState = entry.getValue();
        if (kDebug)
            System.out.println("prior state " + priorState.x.get(0, 0));

        Entry<Double, Matrix<Inputs, N1>> priorUEntry = m_control_history.floor(stateTimeS);
        if (priorUEntry == null)
            priorUEntry = Map.entry(0.0, m_initialControl);
        Matrix<Inputs, N1> priorU = priorUEntry.getValue();
        // integrate to the current time with the previous u
        double timeToNow = currentTimeSec - stateTimeS;
        return m_predictor.predictWithNoise(priorState, priorU, timeToNow);
    }

    /**
     * Finds the feedforward output, u_FF, for the given time.
     */
    public Matrix<Inputs, N1> calculateFeedforward(double ffTimeSec) {
        Matrix<States, N1> ffReference = m_reference.getR(ffTimeSec);
        Matrix<States, N1> ffRDot = m_reference.getRDot(ffTimeSec);
        return m_feedforward.calculateWithRAndRDot(ffReference, ffRDot);
    }

    /**
     * Predicts the future state with only feedforward.
     */
    public RandomVector<States> predictFutureUsingFF(RandomVector<States> initialState, Matrix<Inputs, N1> uff,
            double timeSpanSec) {
        return m_predictor.predictWithNoise(initialState, uff, timeSpanSec);
    }

    /**
     * Finds the feedback output, u_FB, for the given time and predicted state.
     */
    public Matrix<Inputs, N1> calculateFeedback(double actuationTimeSec, RandomVector<States> predicted) {
        // make another reference for the end of the period
        Matrix<States, N1> nextReference = m_reference.getR(actuationTimeSec);
        // now fill the gap between the prediction and the reference
        return m_feedback.calculate(predicted, nextReference);
    }

    /**
     * Record the actual total u applied to the actuators.
     */
    public void recordHistory(double timeSec, Matrix<Inputs, N1> u) {
        m_control_history.put(timeSec, u);
    }
}
