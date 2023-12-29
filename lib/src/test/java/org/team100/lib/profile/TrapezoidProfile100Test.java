package org.team100.lib.profile;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

class TrapezoidProfile100Test {
    private static final double kDt = 0.01;
    boolean dump = true;
    private static final double kDelta = 0.001;

    @Test
    void testProfile() {
        Constraints c = new Constraints(5, 2);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0, 0);
        final State end = new State(1, 0);
        for (double t = 0; t < 2; t += 0.02) {
            sample = profileX.calculate(0.02, end, sample);

            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            if (profileX.isFinished())
                break;
        }
        assertTrue(profileX.isFinished());
    }

    // heading away from the goal, overshoot works correctly.
    @Test
    void testProfile2() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0.1, 1);
        final State end = new State(0, 0);
        for (double t = 0; t < 10; t += 0.02) {
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            sample = profileX.calculate(0.02, end, sample);

            if (profileX.isFinished())
                break;
        }
        assertTrue(profileX.isFinished());
    }

    // heading too fast towards the goal, it jumps the position back at infinite
    // speed and jumps the velocity down at infinite acceleration, to end at the
    // goal.
    @Test
    void testProfile3() {
        Constraints c = new Constraints(5, 1);
        TrapezoidProfile100 profileX = new TrapezoidProfile100(c);
        State sample = new State(0.1, 1);
        final State end = new State(0, 0);
        for (double t = 0; t < 10; t += 0.02) {
            Util.printf("%f %f %f\n", t, sample.position, sample.velocity);
            sample = profileX.calculate(0.02, end, sample);

            if (profileX.isFinished())
                break;
        }
        assertTrue(profileX.isFinished());
    }

    //////////////////////////////////////////////////////

    // Tests below are from WPILib TrapezoidProfileTest.

    /**
     * Asserts "val1" is less than or equal to "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     */
    private static void assertLessThanOrEquals(double val1, double val2) {
        assertTrue(val1 <= val2, val1 + " is greater than " + val2);
    }

    private static void assertNear(double val1, double val2, double eps) {
        assertEquals(val1, val2, eps);
    }

    /**
     * Asserts "val1" is less than or within "eps" of "val2".
     *
     * @param val1 First operand in comparison.
     * @param val2 Second operand in comparison.
     * @param eps  Tolerance for whether values are near to each other.
     */
    private static void assertLessThanOrNear(double val1, double val2, double eps) {
        if (val1 <= val2) {
            assertLessThanOrEquals(val1, val2);
        } else {
            assertNear(val1, val2, eps);
        }
    }

    @Test
    void reachesGoal() {
        Constraints constraints = new Constraints(1.75, 0.75);
        State goal = new State(3, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 450; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(state, goal);
    }

    // Tests that decreasing the maximum velocity in the middle when it is already
    // moving faster than the new max is handled correctly
    @Test
    void posContinuousUnderVelChange() {
        Constraints constraints = new Constraints(1.75, 0.75);
        State goal = new State(12, 0);

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints);
        State state = profile.calculate(kDt, goal, new State());

        double lastPos = state.position;
        for (int i = 0; i < 1600; ++i) {
            if (i == 400) {
                constraints = new Constraints(0.75, 0.75);
                profile = new TrapezoidProfile100(constraints);
            }

            state = profile.calculate(kDt, goal, state);
            double estimatedVel = (state.position - lastPos) / kDt;

            if (i >= 400) {
                // Since estimatedVel can have floating point rounding errors, we check
                // whether value is less than or within an error delta of the new
                // constraint.
                assertLessThanOrNear(estimatedVel, constraints.maxVelocity, 1e-4);

                assertLessThanOrEquals(state.velocity, constraints.maxVelocity);
            }

            lastPos = state.position;
        }
        assertEquals(state, goal);
    }

    // There is some somewhat tricky code for dealing with going backwards
    @Test
    void backwards() {
        Constraints constraints = new Constraints(0.75, 0.75);
        State goal = new State(-2, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 400; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(state, goal);
    }

    @Test
    void switchGoalInMiddle() {
        Constraints constraints = new Constraints(0.75, 0.75);
        State goal = new State(-2, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertNotEquals(state, goal);

        goal = new State(0.0, 0.0);
        profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 550; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(state, goal);
    }

    // Checks to make sure that it hits top speed
    @Test
    void topSpeed() {
        Constraints constraints = new Constraints(0.75, 0.75);
        State goal = new State(4, 0);
        State state = new State();

        TrapezoidProfile100 profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 200; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertNear(constraints.maxVelocity, state.velocity, 10e-5);

        profile = new TrapezoidProfile100(constraints);
        for (int i = 0; i < 2000; ++i) {
            state = profile.calculate(kDt, goal, state);
        }
        assertEquals(state, goal);
    }

}
