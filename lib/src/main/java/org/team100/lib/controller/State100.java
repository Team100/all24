package org.team100.lib.controller;

import org.team100.lib.profile.MotionState;

/**
 * One-dimensional system state, used for measurement and reference.
 * 
 * Includes position, velocity, and acceleration.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 */
public class State100 {
    private final double m_x;
    private final double m_v;
    private final double m_a;

    /** Specify position, velocity, and acceleration. */
    public State100(double x, double v, double a) {
        m_x = x;
        m_v = v;
        m_a = a;
    }

    public State100(MotionState state) {
        m_x = state.getX();
        m_v = state.getV();
        m_a = state.getA();
    }

    public double x() {
        return m_x;
    }

    public double v() {
        return m_v;
    }

    public double a() {
        return m_a;
    }

    public String toString() {
        return String.format("State100(X: %5.3f, V: %5.3f, A: %5.3f)", m_x, m_v, m_a);
    }
}
