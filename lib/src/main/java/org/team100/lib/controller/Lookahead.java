package org.team100.lib.controller;

/**
 * A utility class for interpolating lookahead distance based on current speed.
 */
public class Lookahead {
    public final double min_distanceM;
    public final double max_distanceM;
    public final double min_speedM_S;
    public final double max_speedM_S;

    protected final double delta_distanceM;
    protected final double delta_speedM_S;

    public Lookahead(double min_distanceM, double max_distanceM, double min_speedM_S, double max_speedM_S) {
        this.min_distanceM = min_distanceM;
        this.max_distanceM = max_distanceM;
        this.min_speedM_S = min_speedM_S;
        this.max_speedM_S = max_speedM_S;
        delta_distanceM = max_distanceM - min_distanceM;
        delta_speedM_S = max_speedM_S - min_speedM_S;
    }

    public double getLookaheadForSpeed(double speed) {
        double lookahead = delta_distanceM * (speed - min_speedM_S) / delta_speedM_S + min_distanceM;
        return Double.isNaN(lookahead) ? min_distanceM : Math.max(min_distanceM, Math.min(max_distanceM, lookahead));
    }
}