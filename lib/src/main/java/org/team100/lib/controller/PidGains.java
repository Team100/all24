package org.team100.lib.controller;

public class PidGains {
    public final double p;
    public final double i;
    public final double d;
    public final double integratorRange;
    public final double tolerance;
    public final boolean continuous;

    public PidGains(double p, double i, double d, double range, double tolerance, boolean continuous) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.integratorRange = range;
        this.tolerance = tolerance;
        this.continuous = continuous;
    }
}
