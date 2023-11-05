package org.team100.lib.system;

import org.team100.lib.controller.FeedbackControl;
import org.team100.lib.controller.InversionFeedforward;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.math.RandomVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N1;

/**
 * a better version of the loop thing.
 * 
 * this should do two completely asynchronous things
 * 
 * 1. accept measurements, whenever, timestamped.
 * 2. when you want to actuate something at some time, find the oldest
 * measurement you haven't handled, find the state estimate from that time, and
 * replay all the measurements you have, predicting in between. then finally
 * predict one more time to the chosen actuation time. then calculate the total
 * control output (system input) to achieve the refernce.
 */
public class NonlinearSystemLoop<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final NonlinearPlant<States, Inputs, Outputs> m_plant;
    private final FeedbackControl<States, Inputs, Outputs> m_controller;
    private final InversionFeedforward<States, Inputs, Outputs> m_feedforward;
    private final ExtrapolatingEstimator<States, Inputs, Outputs> m_predictor;
    private final PointEstimator<States, Inputs, Outputs> m_pointEstimator;
    private final LinearPooling<States> m_pooling;

    /**
     * Constructs a state-space loop with the given controller, feedforward, and
     * observer. By default, the initial reference is all zeros. Users should call
     * reset with the initial system state before enabling the loop.
     *
     * @param plant       The system to control.
     * @param controller  State-space controller.
     * @param feedforward Plant inversion feedforward.
     * @param estimator   State-space estimator.
     */
    public NonlinearSystemLoop(
            NonlinearPlant<States, Inputs, Outputs> plant,
            ExtrapolatingEstimator<States, Inputs, Outputs> predictor,
            PointEstimator<States, Inputs, Outputs> pointEstimator,
            LinearPooling<States> pooling,
            FeedbackControl<States, Inputs, Outputs> controller,
            InversionFeedforward<States, Inputs, Outputs> feedforward) {
        m_plant = plant;
        m_predictor = predictor;
        m_pointEstimator = pointEstimator;
        m_pooling = pooling;
        m_controller = controller;
        m_feedforward = feedforward;
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     * 
     * TODO: allow time travel, measurement from the past.
     * 
     * @param x state to fuse with, get rid of this
     * @param y      measurement
     */
    public RandomVector<States> correct(RandomVector<States> x, RandomVector<Outputs> y) {
        RandomVector<States> xx = m_pointEstimator.stateForMeasurementWithZeroU(y);
        return m_pooling.fuse(xx, x);
    }

    /**
     * integrate forward dt.
     * TODO: use absolute time
     */
    public RandomVector<States> predictState(RandomVector<States> initial, Matrix<Inputs, N1> calculatedU,
            double dtSeconds) {
        return m_predictor.predictWithNoise(initial, calculatedU, dtSeconds);
    }

    /**
     * find controller output to get to reference at dt; uses observer xhat.
     * TODO: use absolute time
     */
    public Matrix<Inputs, N1> calculateTotalU(RandomVector<States> xhat, Matrix<States, N1> r, Matrix<States, N1> rDot,
            double dtSeconds) {
        Matrix<Inputs, N1> controllerU = m_controller.calculate(xhat, r);
        Matrix<Inputs, N1> feedforwardU = m_feedforward.calculateWithRAndRDot(r, rDot);
        return m_plant.limit(controllerU.plus(feedforwardU));
    }
}
