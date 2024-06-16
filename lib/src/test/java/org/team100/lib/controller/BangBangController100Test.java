package org.team100.lib.controller;

import java.util.Deque;
import java.util.LinkedList;

import org.junit.jupiter.api.Test;
import org.team100.lib.profile.Profile100;
import org.team100.lib.profile.TrapezoidProfile100;

class BangBangController100Test {
    private static final double kDt = 0.02;

    @Test
    void testChatter() {
        // try to produce chatter, set tolerance to zero
        final Profile100 profile = new TrapezoidProfile100(1, 1, 0);
        final State100 goal = new State100();
        final State100 initial = new State100(1, 0);
        // measurements are substantially delayed.
        final Deque<State100> queue = fill(initial, 0.1);

        for (int i = 0; i < 500; ++i) {
            State100 delayedMeasurement = queue.removeFirst();
            State100 u = profile.calculate(kDt, delayedMeasurement, goal);
            State100 currentMeasurement = queue.getLast();
            State100 newState = integrate(currentMeasurement, u.a());
            queue.addLast(newState);
            System.out.printf("%5.3f, %5.3f, %5.3f\n", newState.x(), newState.v(), newState.a());
        }
    }

    /** Create and initialize the delay queue. */
    private Deque<State100> fill(State100 initial, double delay) {
        Deque<State100> queue = new LinkedList<>();
        for (int i = 0; i < (int) (delay / kDt); ++i) {
            queue.addLast(initial);
        }
        return queue;
    }

    /** The system is a double integrator. */
    private static State100 integrate(State100 currentMeasurement, double a) {
        double x = currentMeasurement.x() + currentMeasurement.v() * kDt + 0.5 * a * Math.pow(kDt, 2);
        double v = currentMeasurement.v() + a * kDt;
        return new State100(x, v, a);
    }

}
