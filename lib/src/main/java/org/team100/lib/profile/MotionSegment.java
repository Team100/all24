package org.team100.lib.profile;

public class MotionSegment {
    private final MotionState start;
    private final double dt;

    /**
     * Segment of a motion profile with constant acceleration.
     *
     * @param start start motion state
     * @param dt    time delta
     */
    public MotionSegment(MotionState start, double dt) {
        this.start = start;
        this.dt = dt;
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    MotionState get(double t) {
        return start.get(t);
    }

    /**
     * Returns the [MotionState] at the end of the segment (time [dt]).
     */
    MotionState end() {
        return start.get(dt);
    }

    /**
     * Returns a reversed version of the segment. Note: it isn't possible to reverse
     * a segment completely so this
     * method only guarantees that the start and end velocities will be swapped.
     */
    MotionSegment reversed() {
        MotionState end = end();
        MotionState state = new MotionState(end.getX(), end.getV(), -end.getA(), end.getJ());
        return new MotionSegment(state, dt);
    }

    /**
     * Returns a flipped (negated) version of the segment.
     */
    MotionSegment flipped() {
        return new MotionSegment(start.flipped(), dt);
    }

    public String toString() {
        return String.format("(%s, %f)", start, dt);
    }

    public MotionState getStart() {
        return start;
    }

    public double getDt() {
        return dt;
    }
}
