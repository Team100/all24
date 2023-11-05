package org.team100.controllib.simulation;

import org.team100.controllib.estimator.NewBitemporalEstimatorController;
import org.team100.controllib.math.RandomVector;
import org.team100.controllib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class RoboRIO {
    private static final boolean debug = false;
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;
    private static final double kSecPerRioLoop = kUsecPerRioLoop * kSecPerUsec;

    private final NewBitemporalEstimatorController<N2, N1, N2> estimator;

    public RoboRIO(DoubleIntegratorRotary1D system,
            NewBitemporalEstimatorController<N2, N1, N2> estimator) {

        this.estimator = estimator;
    }

    /**
     * Run the roborio code if it's the right time.
     * 
     * the goal is to decide what to do for the next quantum, so here we are looking
     * at the future.
     */
    public void step(CompleteState state) {
        if (state.systemTimeMicrosec % kUsecPerRioLoop == 0) {
            if (debug)
                System.out.println("rio step " + state.actualTimeSec());

            state.replayCount = estimator.replay(state.systemTimeMicrosec);

            RandomVector<N2> currentState = estimator.predictNow(state.actualTimeSec());

            // Use the midpoint feedforward as the control for the future estimate.

            Matrix<N1, N1> uff = estimator.calculateFeedforward(state.actualTimeSec() + kSecPerRioLoop / 2);

            double actuationTimeSec = state.actualTimeSec() + kSecPerRioLoop;

            if (debug)
                System.out.println(
                        "integrating from " + state.actualTimeSec() + " to " + actuationTimeSec + " uff "
                                + uff.get(0, 0));

            RandomVector<N2> predicted = estimator.predictFutureUsingFF(currentState, uff, kSecPerRioLoop);

            if (debug)
                System.out.println("estimate position at " + actuationTimeSec + " is " + predicted.x.get(0, 0));

            // this prediction is wrong, it's just for the feedforward.
            // we could make a better prediction but it's not worth the time
            state.predictedPosition = predicted.x.get(0, 0);
            state.predictedVelocity = predicted.x.get(1, 0);
            Matrix<N2, N1> nextReference = estimator.m_reference.getR(actuationTimeSec);
            state.referencePosition = nextReference.get(0, 0);
            state.referenceVelocity = nextReference.get(1, 0);
            if (debug)
                System.out.println("reference position at " + actuationTimeSec + " is " + nextReference.get(0, 0));
            Matrix<N2, N1> nextRDot = estimator.m_reference.getRDot(actuationTimeSec);
            state.referenceAcceleration = nextRDot.get(1, 0);

            Matrix<N2, N1> error = nextReference.minus(predicted.x);
            state.errorPosition = error.get(0, 0);
            state.errorVelocity = error.get(1, 0);
            if (debug)
                System.out.println("error position " + error.get(0, 0));

            Matrix<N1, N1> ufb = estimator.calculateFeedback(actuationTimeSec, predicted);
            if (debug)
                System.out.println("calculate ufb " + ufb.get(0, 0));

            state.ffU = uff.get(0, 0);
            state.fbU = ufb.get(0, 0);
            Matrix<N1, N1> u = ufb.plus(uff);
            state.totalU = u.get(0, 0);

            // the control applies starting now
            estimator.record(state.actualTimeSec(), u);
        }
    }

}
