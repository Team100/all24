package org.team100.lib.controller;

import java.util.LinkedList;
import java.util.Queue;

import org.junit.jupiter.api.Test;

@SuppressWarnings("java:S2699") // no assertions here
class BangBangController100Test {
    private static final double kDt = 0.02;

    @Test
    void testDelayWithAccel() {
        System.out.println("testDelayWithAccel");
        // if actuation uses the acceleration field, then delay causes lag in control
        // (equal to the delay) and oscillation around the goal.
        final BangBangController100 profile = new BangBangController100(1, 1, 0);
        State100 goal = new State100();
        State100 initial = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        for (int i1 = 0; i1 < (int) (delay / kDt); ++i1) {
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
        System.out.println("testDelayWithVelocity");
        // if actuation uses the velocity field only, e.g. as input to the outboard
        // velocity controller, then delay slows the controller by the
        // ratio of the delay and the timestep (!)
        // so definitely don't do this -- it's why the "normal" way to use the profile
        // is to use the previous setpoint, not the measurement, as the initial state.
        final BangBangController100 profile = new BangBangController100(1, 1, 0);
        State100 goal = new State100();
        State100 initial = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        for (int i1 = 0; i1 < (int) (delay / kDt); ++i1) {
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

    @Test
    void testDelayWithClosedLoop() {
        System.out.println("testDelayWithClosedLoop");
        // what we actually do is pass the velocity to the outboard closed-loop velocity
        // controller, and use kV and kA to make a feedforward voltage.

        // max vel = 1 rad/s
        // max accel = 1 rad/s^2

        final BangBangController100 profile = new BangBangController100(1, 1, 0);
        State100 goalRad = new State100();
        // just use rotation for now
        State100 initialRad = new State100(1, 0);

        // measurements are substantially delayed.
        Queue<State100> queue = new LinkedList<>();
        double delay = 0.1;
        for (int i1 = 0; i1 < (int) (delay / kDt); ++i1) {
            queue.add(initialRad);
        }

        State100 actualCurrentStateRad = initialRad;

        for (int i = 0; i < 500; ++i) {
            State100 delayedMeasurementRad = queue.remove();
            State100 u = profile.calculate(kDt, delayedMeasurementRad, goalRad);
            double tSec = i * kDt;
            State100 newStateRad = closedLoop(tSec, actualCurrentStateRad, u);
            queue.add(newStateRad);
            actualCurrentStateRad = newStateRad;
        }
    }

    @Test
    void testUnderdrive() {
        System.out.println("testUnderdrive");
        // underdriving should create chatter.
    }

    @Test
    void testOverdrive() {
        System.out.println("testOverdrive");
        // overdriving should create chatter.
    }

    @Test
    void testNoise() {
        System.out.println("testNoise");
        // noise should create chatter on the switching curve.
    }

    private static State100 applyAccelOnly(double tSec, State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + currentMeasurement.v() * kDt + 0.5 * u.a() * Math.pow(kDt, 2);
        double v = currentMeasurement.v() + u.a() * kDt;
        System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, u.a());
        return new State100(x, v, u.a());
    }

    private static State100 applyVelocityOnly(double tSec, State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + u.v() * kDt;
        double v = u.v();
        System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, 0.0);
        return new State100(x, v, 0);
    }

    /**
     * Behave like the Falcon closed-loop velocity controller using VelocityVoltage
     * control. ignores friction.
     */
    private static State100 closedLoop(double t, State100 currentMeasurementRad, final State100 u) {
        // volts per rev/s. about 10 volts per 100 rev/s = 0.1;
        final double kV = 0.1;
        // volts per rev/s^2. about 1 Nm per rev/s^2, about 50 A per Nm, about 1.25
        // volts
        // this is a pretty high number, because the flywheel is pretty big and heavy!
        final double kA = 1.25;
        // saturate at about 10 rev/s error
        final double kP = 1.0;

        double u_FFVolts = kV * u.v() + kA * u.a();

        // kraken
        final double kStallTorqueNm = 7.09;
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
            double tSec = t + i * dt;
            double speedRad_S = resultRad.v();
            double errorRad_S = u.v() - speedRad_S;
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
            double a = torqueNm / kMomentOfInertiaKgM2;
            double v = speedRad_S + a * dt;
            double x = resultRad.x() + speedRad_S * dt + 0.5 * a * dt * dt;
            // System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", tSec, x, v, a);

            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f,, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f,, %5.3f, %5.3f\n",
            tSec, x, v, a,
            u_FFVolts, u_FBVolts, u_TOTALVolts, netVolts, currentAmps,
            u.v(), u.a());

            resultRad = new State100(x, v, a);
        }

        return resultRad;

    }

}
