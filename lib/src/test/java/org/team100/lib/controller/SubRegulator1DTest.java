package org.team100.lib.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionProfileGenerator;
import org.team100.lib.profile.MotionState;
import org.team100.lib.system.examples.FrictionCartesian1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

class SubRegulator1DTest {
    private static final double kDt = 0.02;
    private static final double kDelta = 0.01;

    /**
     * Demonstrates that the controller output is zero if the current and desired
     * states are the same.
     */
    @Test
    void testAtSetpoint() {
        Vector<N2> stateTolerance_x = VecBuilder.fill(0.02, 0.02);
        Vector<N1> controlTolerance_x = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        FrictionCartesian1D system_x = new FrictionCartesian1D(wx, vx);
        SubRegulator1D regulator = new SubRegulator1D(system_x, stateTolerance_x, controlTolerance_x);

        Variance<N2> px = Variance.from2StdDev(.01, .01);
        RandomVector<N2> xhat_x = new RandomVector<>(VecBuilder.fill(0, 0), px);
        Vector<N2> r_x = VecBuilder.fill(0, 0);
        Vector<N2> rDot_x = VecBuilder.fill(0, 0);

        Matrix<N1, N1> output = regulator.calculateTotalU(xhat_x, r_x, rDot_x, kDt);

        assertEquals(0, output.get(0, 0), kDelta);
    }

    @Test
    void testFictionCartesian1DAtSetpoint() {
        Vector<N2> stateTolerance_x = VecBuilder.fill(0.02, 0.02);
        Vector<N1> controlTolerance_x = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        FrictionCartesian1D system_x = new FrictionCartesian1D(wx, vx);
        SubRegulator1D regulator = new SubRegulator1D(system_x, stateTolerance_x, controlTolerance_x);

        Variance<N2> px = Variance.from2StdDev(.01, .01);
        RandomVector<N2> xhat_x = new RandomVector<>(VecBuilder.fill(0, 0), px);
        Vector<N2> r_x = VecBuilder.fill(0, 0);
        Vector<N2> rDot_x = VecBuilder.fill(0, 0);

        Matrix<N1, N1> output = regulator.calculateTotalU(xhat_x, r_x, rDot_x, kDt);

        assertEquals(0, output.get(0, 0), kDelta);
    }

    @Test
    void driveOneMeter() {
        MotionProfile profileX = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState((double) 0, 0),
                new MotionState((double) 1, 0),
                5,
                2,
                0);

        double duration = profileX.duration();
        assertEquals(1.414, duration, kDelta);

        MotionState sample = new MotionState(0, 0);

        Vector<N2> stateTolerance_x = VecBuilder.fill(0.02, 0.02);
        Vector<N1> controlTolerance_x = VecBuilder.fill(.001);
        WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
        MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
        // this is a double integrator with friction
        FrictionCartesian1D system_x = new FrictionCartesian1D(wx, vx);
        SubRegulator1D regulator = new SubRegulator1D(system_x, stateTolerance_x, controlTolerance_x);

        Variance<N2> px = Variance.from2StdDev(.01, .01);

        RandomVector<N2> xhat_x = new RandomVector<N2>(VecBuilder.fill(0, 0), px);
        Vector<N2> r_x = VecBuilder.fill(0, 0);

        double actual = 0;
        double actual_v = 0;
        double time = 0;
        // go past the end just to get to exactly-zero velocity.
        while (time <= duration + 1) {

            // sample at the current instant
            sample = profileX.get(time);

            // correct estimate using the actual forecast from last iteration
            xhat_x = regulator.correctPosition(xhat_x, actual);
            xhat_x = regulator.correctVelocity(xhat_x, actual_v);

            // references for the current instant
            r_x = VecBuilder.fill(sample.getX(), sample.getV());
            Vector<N2> rDot_x = VecBuilder.fill(sample.getV(), sample.getA());

            // calculate controller output
            Matrix<N1, N1> output = regulator.calculateTotalU(xhat_x, r_x, rDot_x, kDt);

            // integrate to predict x in the future
            xhat_x = regulator.predictState(xhat_x, output, kDt);

            // forecast actual behavior, lossy double integrator
            actual += actual_v * kDt;
            actual_v += (output.get(0, 0) - actual_v) * kDt;
            System.out.printf("t: %f, sample x %f v %f, actual %f, xhat x %f v %f, output %f\n",
                    time,
                    sample.getX(),
                    sample.getV(),
                    actual,
                    xhat_x.x.get(0, 0),
                    xhat_x.x.get(1, 0),
                    output.get(0, 0));
            time += kDt;
        }

        // the profile gets to the goal, kinda
        assertEquals(1, sample.getX(), kDelta);
        assertEquals(0, sample.getV(), kDelta);

        // the current pose ends up at the goal.
        // it actually overshoots a little bit, i think because the
        // feedforward is pushing pretty hard until the very end.
        // in any case it's pretty close.
        assertEquals(1.03, actual, kDelta);

        // the prediction ends up near the goal, exactly the same
        // as the "actual" above.
        assertEquals(1.03, xhat_x.x.get(0, 0), kDelta);
        assertEquals(0, xhat_x.x.get(1, 0), kDelta);

        // the reference ends up exactly at the goal
        assertEquals(1, r_x.get(0, 0), kDelta);
        assertEquals(0, r_x.get(1, 0), kDelta);
    }
}
