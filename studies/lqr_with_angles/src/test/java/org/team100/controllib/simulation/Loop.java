package org.team100.controllib.simulation;

import org.team100.controllib.controller.GainCalculator;
import org.team100.controllib.estimator.NewBitemporalEstimatorController;
import org.team100.controllib.math.AngularRandomVector;
import org.team100.controllib.math.MeasurementUncertainty;
import org.team100.controllib.math.Variance;
import org.team100.controllib.math.WhiteNoiseVector;
import org.team100.controllib.system.examples.DoubleIntegratorRotary1D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class Loop {
    // sparkmax is 500us between measurements, maybe try that.
    private static final long kUsecPerSimLoop = 2000; // 2 ms per simulation loop
    private static final long kUsecPerRioLoop = 20000; // 20 ms per rio loop
    private static final double kSecPerUsec = 1e-6;
    private static final double kSecPerRioLoop = kUsecPerRioLoop * kSecPerUsec;

    private final CompleteState state;
    private final NewBitemporalEstimatorController<N2, N1, N2> estimator;
    private final DoubleIntegratorRotary1D system;
    private final PositionSensor positionSensor;
    private final VelocitySensor velocitySensor;
    private final RoboRIO roborio;

    private final Scenario m_scenario;

    public Loop(Scenario scenario) {
        m_scenario = scenario;
        state = new CompleteState();
        WhiteNoiseVector<N2> w = WhiteNoiseVector.noise2(0.015, 0.17);
        MeasurementUncertainty<N2> v = MeasurementUncertainty.for2(0.01, 0.1);
        system = new DoubleIntegratorRotary1D(w, v);

        Vector<N2> stateTolerance = VecBuilder.fill(0.01, 0.01);
        Vector<N1> controlTolerance = VecBuilder.fill(12.0);
        GainCalculator<N2, N1, N2> gc = new GainCalculator<>(system, stateTolerance, controlTolerance,
                kSecPerRioLoop);

        Matrix<N1,N1> initialControl = new Matrix<>(Nat.N1(),Nat.N1());

        estimator = new NewBitemporalEstimatorController<>(
                system,
                makeInitialState(scenario),
                initialControl,
                scenario.reference(),
                gc.getK());

        positionSensor = new PositionSensor(system, estimator);
        velocitySensor = new VelocitySensor(system, estimator);
        roborio = new RoboRIO(system, estimator);
    }

    private AngularRandomVector<N2> makeInitialState(Scenario scenario) {
        // high variance
        Matrix<N2, N2> initP = new Matrix<>(Nat.N2(), Nat.N2());
        initP.set(0, 0, 1e9);
        initP.set(1, 1, 1e9);
        Matrix<N2,N1> initx = scenario.reference().getR(0);
        return new AngularRandomVector<N2>(initx, new Variance<>(initP));
    }

    public void run() {
        Matrix<N2,N1> initr = m_scenario.reference().getR(0);
        Matrix<N2,N1> initrdot = m_scenario.reference().getRDot(0);

        state.init(initr.get(0,0), initr.get(1,0), initrdot.get(1,0));
        System.out.println("\n\n" + m_scenario.label());
        System.out.println(state.header());
        for (long step = 0; step < 2000; ++step) {
            state.systemTimeMicrosec = step * kUsecPerSimLoop; // fpgatime
            updateActual();
            updateObservation();
            positionSensor.step(state);
            velocitySensor.step(state);
            roborio.step(state);
            updateResidual();
            System.out.println(state.toString());
        }
    }

    /**
     * Update the actual state of the physical system.
     */
    void updateActual() {
        Matrix<N2,N1> r = m_scenario.reference().getR(state.actualTimeSec());
        Matrix<N2,N1> rdot = m_scenario.reference().getRDot(state.actualTimeSec());
        state.actualPosition = r.get(0,0);
        state.actualVelocity = r.get(1,0);
        state.actualAcceleration = rdot.get(1,0);
    }

    /**
     * For now, observations are perfect and instantaneous.
     * 
     * TODO: add lag and noise
     */
    void updateObservation() {
        // position sensor does this now
        // state.observedPosition = state.actualPosition;
        // state.observedVelocity = state.actualVelocity;
        state.observedAcceleration = state.actualAcceleration;
    }

    /**
     * Update the residuals.
     * 
     * The residual should be zero at the *next* filter quantum.
     * Negative error means the data is below the prediction i.e. prediction should
     * be more negative.
     */
    void updateResidual() {
        state.residualPosition = MathUtil.angleModulus(state.actualPosition - state.predictedPosition);
        state.residualVelocity = state.actualVelocity - state.predictedVelocity;
    }
}
