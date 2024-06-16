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
            State100 newState = applyAccelOnly(actualCurrentState, u);
            queue.add(newState);
            actualCurrentState = newState;
            System.out.printf("%5.3f, %5.3f, %5.3f\n", newState.x(), newState.v(), newState.a());
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
            State100 newState = applyVelocityOnly(actualCurrentState, u);
            queue.add(newState);
            actualCurrentState = newState;
            System.out.printf("%5.3f, %5.3f, %5.3f\n", newState.x(), newState.v(), newState.a());
        }
    }

    @Test
    void testDelayWithClosedLoop() {
        System.out.println("testDelayWithClosedLoop");
        // what we actually do is pass the velocity to the outboard closed-loop velocity
        // controller, and use kV and kA to make a feedforward voltage.

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
            State100 newState = closedLoop(actualCurrentState, u);
            queue.add(newState);
            actualCurrentState = newState;

            System.out.printf("%5.3f, %5.3f, %5.3f\n", newState.x(), newState.v(), newState.a());
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

    private static State100 applyAccelOnly(State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + currentMeasurement.v() * kDt + 0.5 * u.a() * Math.pow(kDt, 2);
        double v = currentMeasurement.v() + u.a() * kDt;
        return new State100(x, v, u.a());
    }

    private static State100 applyVelocityOnly(State100 currentMeasurement, State100 u) {
        double x = currentMeasurement.x() + u.v() * kDt;
        double v = u.v();
        return new State100(x, v, 0);
    }

    /**
     * Behave like the Falcon closed-loop velocity controller using VelocityVoltage
     * control.  ignores friction.
     */
    private static State100 closedLoop(State100 currentMeasurement, State100 u) {
        final double kV = 0.1;
        final double kA = 0.1;
        final double kP = 0.2;

        double ff = kV * u.v() * kA * u.a();

        // main loop is 50 hz, outboard loop is 1000 hz, so this runs 20 times.
        double dt = 0.001;
        State100 result = currentMeasurement;
        for (int i = 0; i < 20; ++i) {
            double goal = u.v();
            double measurement = result.v();
            double error = goal - measurement;

        }

        return result;

    }

}
