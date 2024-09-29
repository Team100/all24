package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

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

    public State100(double x, double v) {
        this(x, v, 0);
    }

    public State100() {
        this(0, 0, 0);
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

    public boolean near(State100 other, double tolerance) {
        return MathUtil.isNear(m_x, other.m_x, tolerance) &&
                MathUtil.isNear(m_v, other.m_v, tolerance);
    }

    public String toString() {
        return String.format("State100(X %5.3f V %5.3f A %5.3f)", m_x, m_v, m_a);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof State100) {
            State100 rhs = (State100) other;
            return this.m_x == rhs.m_x && this.m_v == rhs.m_v;
        } else {
            return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_v);
    }

}
