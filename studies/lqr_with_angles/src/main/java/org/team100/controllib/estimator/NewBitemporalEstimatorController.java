package org.team100.controllib.estimator;

import java.util.Map;
import java.util.Map.Entry;

import org.team100.controllib.controller.FeedbackControl;
import org.team100.controllib.controller.InversionFeedforward;
import org.team100.controllib.fusion.LinearPooling;
import org.team100.controllib.fusion.VarianceWeightedLinearPooling;
import org.team100.controllib.math.RandomVector;
import org.team100.controllib.reference.Reference;
import org.team100.controllib.storage.BitemporalBuffer;
import org.team100.controllib.storage.EditableHistory;
import org.team100.controllib.storage.History;
import org.team100.controllib.system.NonlinearPlant;

import java.util.NavigableMap;

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
public class NewBitemporalEstimatorController<States extends Num, Inputs extends Num, Outputs extends Num> {
    private static final boolean debug = false;

    public final RandomVector<States> initialState;
    public final Matrix<Inputs, N1> initialControl;
    public final Reference<States> m_reference;
    // measurements are bitemporal so we can notice late-arriving ones
    public final BitemporalBuffer<RandomVector<Outputs>> m_measurements;
    // we rewrite recent state history as needed.
    public final EditableHistory<RandomVector<States>> m_estimates;
    // control history is immutable.
    public final History<Matrix<Inputs, N1>> m_control_history;
    public final ExtrapolatingEstimator<States, Inputs, Outputs> predictor;
    public final PointEstimator<States, Inputs, Outputs> pointEstimator;
    public final TrendEstimator<States, Inputs, Outputs> trendEstimator;
    public final LinearPooling<States> pooling;
    public final InversionFeedforward<States, Inputs, Outputs> feedforward;
    public final FeedbackControl<States, Inputs, Outputs> feedback;

    // the last recordTime we've seen from the buffer
    private long recordTime;

    /**
     * @param system plant dynamics
     * @param initialState used by the predictor
     * @param initialControl used by the predictor
     * @param reference produces trajectory
     * @param K feedback gain
     */
    public NewBitemporalEstimatorController(
            NonlinearPlant<States, Inputs, Outputs> system,
            RandomVector<States> initialState,
            Matrix<Inputs, N1> initialControl,
            Reference<States> reference,
            Matrix<Inputs, States> K) {
        this.initialState = initialState;
        this.initialControl = initialControl;
        m_reference = reference;
        m_measurements = new BitemporalBuffer<>(1000);
        m_estimates = new EditableHistory<>(1000);
        m_control_history = new History<>(1000);
        predictor = new ExtrapolatingEstimator<>(system);
        pointEstimator = new PointEstimator<>(system);
        trendEstimator = new TrendEstimator<>(system);
        pooling = new VarianceWeightedLinearPooling<>();
        feedforward = new InversionFeedforward<>(system);
        feedback = new FeedbackControl<>(system, K);
    }

    /**
     * Accept a measurement about some time in the past. These are collected later
     * by replay.  Threadsafe.
     */
    public void acceptMeasurement(long recordTimeUs, double validTimeSec, RandomVector<Outputs> measurement) {
        m_measurements.put(recordTimeUs, validTimeSec, measurement);
    }

    /** Update the state history with any measurements that are pending. */
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
        if (debug)
            System.out.println("replay " + todo.size());
        for (Entry<Double, Entry<Long, RandomVector<Outputs>>> measurementEntry : todo.entrySet()) {
            replayCount += 1;
            // find the time of the measurement
            double measurementTime = measurementEntry.getKey();
            // find the most-recent state earlier than the measurement
            Entry<Double, RandomVector<States>> entry = m_estimates.floor(measurementTime);
            if (entry == null) {
                entry = Map.entry(0.0, initialState);
            }
            double stateTimeS = entry.getKey();
            RandomVector<States> priorState = entry.getValue();
            if (debug)
                System.out.println("found position " + priorState.x.get(0, 0));

            // this is the control in use at the time of the prior state
            Entry<Double, Matrix<Inputs, N1>> historicalUEntry = m_control_history.floor(stateTimeS);
            if (historicalUEntry == null)
                historicalUEntry = Map.entry(0.0, initialControl);
            Matrix<Inputs, N1> historical_u = historicalUEntry.getValue();
            if (debug)
                System.out.println("found u " + historical_u);

            // there could be multiple controls between the state and the measurement.
            // so make a new state for each control
            NavigableMap<Double, Matrix<Inputs, N1>> uEntries = m_control_history.validSubMap(stateTimeS,
                    measurementTime);
            for (Entry<Double, Matrix<Inputs, N1>> uEntry : uEntries.entrySet()) {
                // integrate the prior state and the prior u up to the new u
                double endS = uEntry.getKey();
                double integrationSpanS = endS - stateTimeS;
                if (debug)
                    System.out
                            .println("tween integrating from " + stateTimeS + " to " + endS + " u " + historical_u);
                if (debug)
                    System.out.println("tween prior state " + priorState.x.get(0, 0));
                priorState = predictor.predictWithNoise(
                        priorState,
                        historical_u,
                        integrationSpanS);
                if (debug)
                    System.out.println("tween estimate position " + priorState.x.get(0, 0));

                historical_u = uEntry.getValue();
                stateTimeS = endS;
            }
            // now the prior state and state time are up to the most-recent change in u
            // and the historical u is also the latest u
            // so integrate the last little bit
            if (debug)
                System.out.println(
                        "replay integrating from " + stateTimeS + " to " + measurementTime + " u " + historical_u);
            if (debug)
                System.out.println("replay prior state " + priorState.x.get(0, 0));
            double stateToMeasurementS = measurementTime - stateTimeS;
            RandomVector<States> predictedState = predictor.predictWithNoise(
                    priorState,
                    historical_u,
                    stateToMeasurementS);
            if (debug)
                System.out.println("replay estimate " + predictedState.x.get(0, 0));

            // this is the measurement state
            RandomVector<States> measurementState = pointEstimator
                    .stateForMeasurementWithZeroU(measurementEntry.getValue().getValue());
            if (debug)
                System.out.println("replay measurement " + measurementState.x.get(0, 0));
            // pool the measurement and the extrapolation
            RandomVector<States> fused = pooling.fuse(predictedState, measurementState);
            // record the new estimate
            if (debug)
                System.out.println("replay fused " + fused.x.get(0, 0));
            m_estimates.put(measurementTime, fused);
        }
        return replayCount;
    }

    /** Predict the state for the current instant. */
    public RandomVector<States> predictNow(double currentTimeSec) {
        Entry<Double, RandomVector<States>> entry = m_estimates.floor(currentTimeSec);
        if (entry == null)
            entry = Map.entry(0.0, initialState);

        double stateTimeS = entry.getKey();
        RandomVector<States> priorState = entry.getValue();
        if (debug)
            System.out.println("prior state " + priorState.x.get(0, 0));

        Entry<Double, Matrix<Inputs, N1>> priorUEntry = m_control_history.floor(stateTimeS);
        if (priorUEntry == null)
            priorUEntry = Map.entry(0.0, initialControl);
        Matrix<Inputs, N1> priorU = priorUEntry.getValue();
        // integrate to the current time with the previous u
        double timeToNow = currentTimeSec - stateTimeS;
        return predictor.predictWithNoise(priorState, priorU, timeToNow);
    }

    public Matrix<Inputs, N1> calculateFeedforward(double ffTimeSec) {
        Matrix<States, N1> ffReference = m_reference.getR(ffTimeSec);
        Matrix<States, N1> ffRDot = m_reference.getRDot(ffTimeSec);
        Matrix<Inputs, N1> uff = feedforward.calculateWithRAndRDot(ffReference, ffRDot);
        return uff;
    }

    /** Predict the future state with only feedforward. */
    public RandomVector<States> predictFutureUsingFF(RandomVector<States> initialState, Matrix<Inputs, N1> uff,
            double timeSpanSec) {
        return predictor.predictWithNoise(initialState, uff, timeSpanSec);
    }

    public Matrix<Inputs, N1> calculateFeedback(double actuationTimeSec, RandomVector<States> predicted) {
        // make another reference for the end of the period
        Matrix<States, N1> nextReference = m_reference.getR(actuationTimeSec);
        // now fill the gap between the prediction and the reference
        return feedback.calculate(predicted, nextReference);
    }

    /** Record the actual u applied to the actuators. */
    public void record(double timeSec, Matrix<Inputs, N1> u) {
        m_control_history.put(timeSec, u);

    }
}
