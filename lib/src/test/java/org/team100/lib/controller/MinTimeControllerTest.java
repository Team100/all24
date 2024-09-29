package org.team100.lib.controller;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;
import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.TestLogger;
import org.team100.lib.state.State100;
import org.team100.lib.logging.SupplierLogger2;

import edu.wpi.first.math.MathUtil;

/**
 * Graphs for these tests are located here
 * 
 * https://docs.google.com/spreadsheets/d/1iBkfx08k89OAlbCgJ4-BDdx4meIBZ949ViCQly79_6M
 */
@SuppressWarnings("java:S2699") // no assertions here
class MinTimeControllerTest {
    private static final double kDt = 0.02;
    private static final SupplierLogger2 logger = new TestLogger().getSupplierLogger();

    private final Random rand = new Random();

    @Test
    void testDelayWithAccel() {
        // System.out.println("testDelayWithAccel");
        // if actuation uses the acceleration field, then delay causes lag in control
        // (equal to the delay) and oscillation around the goal.
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                1, // switchingA
                0.9, // weakG
                1.1, // strongI
                0, // tolerance
                0.1, // finish
                new double[] { 10.0, 10.0 } // k
        );
        State100 goal = new State100();
        State100 initial = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initial);
        }

        State100 actualCurrentState = initial;

        for (int i = 0; i < 500; ++i) {
            State100 delayedMeasurement = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurement, goal);
            double tSec = i * kDt;
            State100 newState = applyAccelOnly(tSec, actualCurrentState, u);
            queue.add(newState);
            actualCurrentState = newState;
        }
    }

    @Test
    void testDelayWithVelocity() {
        // System.out.println("testDelayWithVelocity");
        // if actuation uses the velocity field only, e.g. as input to the outboard
        // velocity controller, then delay slows the controller by the
        // ratio of the delay and the timestep (!)
        // so definitely don't do this -- it's why the "normal" way to use the profile
        // is to use the previous setpoint, not the measurement, as the initial state.
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                1, // switchingA
                0.9, // weakG
                1.1, // strongI
                0, // tolerance
                0.1, // finish
                new double[] { 10.0, 10.0 } // k
        );
        State100 goal = new State100();
        State100 initial = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initial);
        }

        State100 actualCurrentState = initial;

        for (int i = 0; i < 500; ++i) {
            State100 delayedMeasurement = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurement, goal);
            double tSec = i * kDt;
            State100 newState = applyVelocityOnly(tSec, actualCurrentState, u);
            queue.add(newState);
            actualCurrentState = newState;
        }
    }

    /** What happens when we try to go from -pi to pi? */
    @Test
    void testAngleWrapping() {
        // System.out.println("testAngleWrapping");
        final MinTimeController profile = new MinTimeController(
                logger,
                MathUtil::angleModulus,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        // almost -pi
        State100 initialRad = new State100(-3, 0);
        // almost pi
        State100 goalRad = new State100(3, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        // double delay = 0.1;
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 400;
        final double noise = 0.0;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(MathUtil::angleModulus, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    /**
     * This is one of my favorite puzzle cases. We're heading fast in one direction
     * and want to go more than half way around the cirlce in that same direction,
     * so it's better to keep going even though the distance is longer.
     * 
     * The current MinTimeController does not do the right thing in this case.
     * 
     * To do the right thing, it would have to compare the ETAs of the "inside the
     * modulus" path vs the "crossing the modulus" path.
     * 
     * In reality the controller attempts to stop, but the velocity carries the
     * system over boundary anyway, and it speeds up again. It would be better not
     * to have that little pause, but it's not the end of the world. It would be
     * good to clean up the math in MinTimeController so this case works correctly.
     */
    @Test
    void testMovingAngleWrapping() {
        // System.out.println("testMovingAngleWrapping");
        final MinTimeController profile = new MinTimeController(
                logger,
                MathUtil::angleModulus,
                1, // maxV
                0.4, // switchingA
                0.3, // weakG
                0.5, // strongI
                0,
                0.1,
                new double[] { 10.0, 10.0 });
        // the short distance is across pi
        // but we're moving fast the other way.
        State100 initialRad = new State100(1.7, -1);
        State100 goalRad = new State100(-1.7, -1);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        // double delay = 0.1;
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 400;
        final double noise = 0.0;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(MathUtil::angleModulus, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    @Test
    void testDelayWithClosedLoop() {
        // System.out.println("testDelayWithClosedLoop");
        // what we actually do is pass the velocity to the outboard closed-loop velocity
        // controller, and use kV and kA to make a feedforward voltage.

        // motor real max accel is about 1 rad/s^2
        // so to allow some headroom, use 20% less.
        // max vel = 1 rad/s
        // max accel = 0.8 rad/s^2
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        State100 goalRad = new State100();
        // just use rotation for now
        State100 initialRad = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        // double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 400;
        final double noise = 0.0;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(x -> x, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    @Test
    void testMaxVClosedLoop() {
        // System.out.println("testMaxVClosedLoop");

        // motor real max accel is about 1 rad/s^2
        // so to allow some headroom, use 20% less.
        // max vel = 1 rad/s
        // max accel = 0.8 rad/s^2
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                0.6, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        State100 goalRad = new State100();
        // just use rotation for now
        State100 initialRad = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        // double delay = 0.1;
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 400;
        final double noise = 0.0;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(x -> x, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    @Test
    void testMovingGoalDelayWithClosedLoop() {
        // System.out.println("testMovingGoalDelayWithClosedLoop");
        // what we actually do is pass the velocity to the outboard closed-loop velocity
        // controller, and use kV and kA to make a feedforward voltage.

        // motor real max accel is about 1 rad/s^2
        // so to allow some headroom, use 20% less.
        // max vel = 1 rad/s
        // max accel = 0.8 rad/s^2
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        State100 goalRad = new State100(1, 1);
        // just use rotation for now
        State100 initialRad = new State100(0, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        // double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 200;
        final double noise = 0.0;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(x -> x, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    /**
     * Shows what happens when the motor response is significantly less than the
     * model expects, i.e. the expected "profile" is "too fast". Since the system
     * can't come close to the predicted goal path, it overshoots and orbits.
     */
    @Test
    void testUnderdrive() {
        // System.out.println("testUnderdrive");
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        State100 goalRad = new State100();
        State100 initialRad = new State100(1, 0);
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        final int iterations = 400;
        final double noise = 0.0;
        final double drive = 0.5;
        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;

            State100 newStateRad = closedLoop(x -> x, tSec, noise, drive, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    /**
     * Shows what happens when the system responds more quickly than expected. When
     * the system reaches the switching surface, it quickly leaves it, resulting in
     * full-scale chatter on the goal path. Making the "weak G" really weak will
     * help avoid full-scale reversing but it still chatters in the positive
     * direction between strong and weak.
     * 
     * The lesson is that it's better to overestimate the system parameters (leading
     * to overshoot) than to underestimate the parameters (leading to chatter).
     */
    @Test
    void testOverdrive() {
        // System.out.println("testOverdrive");
        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        State100 goalRad = new State100();
        State100 initialRad = new State100(1, 0);
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        final int iterations = 400;
        final double noise = 0.0;
        final double drive = 1.5;
        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;

            State100 newStateRad = closedLoop(x -> x, tSec, noise, drive, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    State100 addNoise(double noise, State100 p) {
        return new State100(addNoise(noise, p.x()), addNoise(noise, p.v()), p.a());
    }

    double addNoise(double noise, Double p) {
        return rand.nextGaussian() * noise + p;
    }

    /**
     * Illustrates noise in both position and velocity measurements.
     * 
     * Shows that the proportional controller in the center needs to be soft to
     * avoid overresponding to the noise. another option would be to filter the
     * noise though that would introduce delay.
     * 
     * The center section also needs to be bigger than the noise scale to avoid
     * full-scale orbiting at the goal.
     */
    @Test
    void testDelayAndNoiseWithClosedLoop() {
        // System.out.println("testDelayAndNoiseWithClosedLoop");
        // what we actually do is pass the velocity to the outboard closed-loop velocity
        // controller, and use kV and kA to make a feedforward voltage.

        // motor real max accel is about 1 rad/s^2
        // so to allow some headroom, use 20% less.
        // max vel = 1 rad/s
        // max accel = 0.8 rad/s^2

        final MinTimeController profile = new MinTimeController(
                logger,
                x -> x,
                1, // maxV
                0.9, // switchingA
                0.8, // weakG
                1.0, // strongI
                0, 0.1, new double[] { 10.0, 10.0 });
        final State100 goalRad = new State100();
        // just use rotation for now
        State100 initialRad = new State100(1, 0);

        // queue contains actual state, not noise.
        Queue<State100> queue = new LinkedList<>();
        // double delay = 0.1;
        double delay = 0.0;
        for (int i1 = 0; i1 < 1 + (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;
        // final int iterations = 500;
        final int iterations = 400;
        // the controller proportional zone needs to be bigger than the noise, to
        // prevent full-scale orbiting
        final double noise = 0.025;

        for (int i = 0; i < iterations; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, addNoise(noise, delayedMeasurementRad), goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(x -> x, tSec, noise, 1.0, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }

    }

    private static State100 applyAccelOnly(double tSec, State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + currentMeasurement.v() * kDt + 0.5 * u.a() * Math.pow(kDt, 2);
        double v = currentMeasurement.v() + u.a() * kDt;
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, u.a());
        return new State100(x, v, u.a());
    }

    private static State100 applyVelocityOnly(double tSec, State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + u.v() * kDt;
        double v = u.v();
        // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, 0.0);
        return new State100(x, v, 0);
    }

    /**
     * Behave like the Falcon closed-loop velocity controller using VelocityVoltage
     * control. ignores friction.
     * 
     * @param drive 1.0 is normal, more is overdrive, less is underdrive.
     */
    private State100 closedLoop(
            DoubleUnaryOperator modulus,
            double t,
            double noise,
            double drive,
            State100 currentMeasurementRad,
            final State100 u) {
        // volts per rev/s. about 10 volts per 100 rev/s = 0.1;
        final double kV = 0.1;
        // volts per rev/s^2.
        // want max accel to be about 1.
        // motor torque limit is about 1 Nm, using about 50 A
        // inertia is per rev/s^2,
        // motor R is 0.025, so to get 50A through the motor takes about 1.25 volts
        // this is a pretty high number, because the flywheel is pretty big and heavy!
        final double kA = 1.25;
        // saturate at about 10 rev/s error
        final double kP = 1.0;

        double u_FFVolts = kV * u.v() + kA * u.a();

        // kraken
        // final double kStallTorqueNm = 7.09;
        final double kFreeSpeedRev_S = 100;
        final double kCurrentLimitA = 50;
        final double kMaxVolts = 12;
        final double kROhms = 0.025;
        final double kTNm_amp = 0.019;
        // max current-limited torque is about 0.95 Nm.
        // 1kg disc, 0.1m radius, iz= 0.5*mr^2
        // 8kg disc, 0.5m radius, I = 0.5*mr^2 = 0.5 * 8 * 0.5 * 0.5 = 1
        final double kMomentOfInertiaKgM2 = 1;

        // main loop is 50 hz, outboard loop is 1000 hz, so this runs 20 times.
        double dt = 0.001;
        State100 resultRad = currentMeasurementRad;
        for (int i = 0; i < 20; ++i) {
            // double tSec = t + i * dt;
            // these are actual states
            double positionRad = resultRad.x();
            double speedRad_S = resultRad.v();

            // this includes noise
            double errorRad_S = u.v() - addNoise(noise, speedRad_S);
            double u_FBVolts = kP * errorRad_S;

            // applied voltage
            double u_TOTALVolts = u_FFVolts + u_FBVolts;

            double backEmfVolts = kMaxVolts * speedRad_S / kFreeSpeedRev_S;
            double netVolts = u_TOTALVolts - backEmfVolts;
            double currentAmps = netVolts / kROhms;
            if (Math.abs(currentAmps) > kCurrentLimitA) {
                currentAmps = Math.signum(currentAmps) * kCurrentLimitA;
            }
            double torqueNm = currentAmps * kTNm_amp;
            // N = kgm/s^2
            // Nm = kgm^2/s^2
            // inertia = kgm^2 = Nm*s^2 = Nm / rad/s^2.
            // torque = I * a.
            // target a is 1, I is 0.005, so target torque is 0.005, hm, that's much lower
            // than the motor is capable of.

            // target accel is 1 rad/s^2 so target torque
            // these are actual state, not noisy measurements.
            // note the fudge factor for over/under drive
            double a = drive * torqueNm / kMomentOfInertiaKgM2;
            double v = speedRad_S + a * dt;
            double x = modulus.applyAsDouble(positionRad + speedRad_S * dt + 0.5 * a * dt * dt);
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, a);

            // System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f,, %6.3f, %6.3f, %6.3f, %6.3f,
            // %6.3f,, %6.3f, %6.3f\n",
            // tSec, x, v, a,
            // u_FFVolts, u_FBVolts, u_TOTALVolts, netVolts, currentAmps,
            // u.v(), u.a());

            resultRad = new State100(x, v, a);
        }

        // return actual state, not noisy measurement.
        return resultRad;

    }

}
