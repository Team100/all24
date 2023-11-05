package org.team100.lib.profile;

/**
 * Kinematic state of a one-dimensional motion profile at any given time.
 * 
 * Includes position, velocity, acceleration, and jerk.
 */
public class MotionState {
    private final double x;
    private final double v;
    private final double a;
    private final double j;

    public MotionState(double x, double v, double a, double j) {
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = j;
    }

    public MotionState(double x, double v, double a) {
        this(x, v, a, 0);
    }

    public MotionState(double x, double v) {
        this(x, v, 0);
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    public MotionState get(double t) {
        return new MotionState(
                x + v * t + a / 2 * t * t + j / 6 * t * t * t,
                v + a * t + j / 2 * t * t,
                a + j * t,
                j);
    }

    /**
     * Returns a flipped (negated) version of the state.
     */
    public MotionState flipped() {
        return new MotionState(-x, -v, -a, -j);
    }

    /**
     * Returns the state with velocity, acceleration, and jerk zeroed.
     */
    public MotionState stationary() {
        return new MotionState(x, 0.0, 0.0, 0.0);
    }

    public String toString() {
        return String.format("(x=%.3f, v=%.3f, a=%.3f, j=%.3f)", x, v, a, j);
    }

    public double getX() {
        return x;
    }

    public double getV() {
        return v;
    }

    public double getA() {
        return a;
    }

    public double getJ() {
        return j;
    }
}
