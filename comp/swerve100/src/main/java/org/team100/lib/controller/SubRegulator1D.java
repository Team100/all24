package org.team100.lib.controller;

import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.system.NonlinearSystemLoop;
import org.team100.lib.system.examples.NoisyLimitedPlant1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional estimator and full state feedback controller with feedforward.
 */
public class SubRegulator1D {
    private static final double kDt = 0.02;

    final Vector<N2> stateTolerance;
    final Vector<N1> controlTolerance;

    public WhiteNoiseVector<N2> w;
    public MeasurementUncertainty<N2> v;

    private NoisyLimitedPlant1D system;

    private GainCalculator<N2, N1, N2> gc;
    private Matrix<N1, N2> K;
    private FeedbackControl<N2, N1, N2> controller;

    private ExtrapolatingEstimator<N2, N1, N2> predictor;
    private PointEstimator<N2, N1, N2> pointEstimator;
    private LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();
    private InversionFeedforward<N2, N1, N2> feedforward;

    public NonlinearSystemLoop<N2, N1, N2> loop;

    public SubRegulator1D(NoisyLimitedPlant1D system, Vector<N2> stateTolerance, Vector<N1> controlTolerance) {
        this.stateTolerance = stateTolerance;
        this.controlTolerance = controlTolerance;
        this.w = system.w();
        this.v = system.v();
        this.system = system;
        gc = new GainCalculator<>(system, stateTolerance, controlTolerance, kDt);
        K = gc.getK();
        // System.out.println(K);
        controller = new FeedbackControl<>(system, K);
        predictor = new ExtrapolatingEstimator<>(system);
        pointEstimator = new PointEstimator<>(system);
        feedforward = new InversionFeedforward<>(system);
        loop = new NonlinearSystemLoop<>(system, predictor, pointEstimator, pooling, controller, feedforward);
    }

    public static Vector<N2> getR(State100 desiredState) {
        return VecBuilder.fill(desiredState.x(), desiredState.v());
    }

    public static Vector<N2> getRDot(State100 desiredState) {
        return VecBuilder.fill(desiredState.v(), desiredState.a());
    }

    /**
     * Correct the state estimate x-hat using the position measurement in y.
     * 
     * @param x state to fuse with, get rid of this
     * @param y measurement
     */
    public RandomVector<N2> correctPosition(RandomVector<N2> x, double y) {
        return loop.correct(x, system.position(y));
    }

    /**
     * Correct the state estimate x-hat using the velocity measurement in y.
     * 
     * @param x state to fuse with, get rid of this
     * @param y measurement
     */
    public RandomVector<N2> correctVelocity(RandomVector<N2> x, double y) {
        return loop.correct(x, system.velocity(y));
    }

    public Matrix<N1, N1> calculateTotalU(RandomVector<N2> xhat, Matrix<N2, N1> r, Matrix<N2, N1> rDot,
            double dtSeconds) {
        return loop.calculateTotalU(xhat, r, rDot, dtSeconds);
    }

    public RandomVector<N2> predictState(RandomVector<N2> initial, Matrix<N1, N1> calculatedU,
            double dtSeconds) {
        return loop.predictState(initial, calculatedU, dtSeconds);
    }
}