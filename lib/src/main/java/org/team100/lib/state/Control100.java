package org.team100.lib.state;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;

/**
 * One-dimensional system state, used for control, so it includes acceleration,
 * which could be part of the control output.
 * 
 * The usual state-space representation would be X = (x,v) and Xdot = (v,a).
 * Units are meters, radians, and seconds.
 */
public class Control100 {
    private final double m_x;
    private final double m_v;
    private final double m_a;

    /** Specify position, velocity, and acceleration. */
    public Control100(double x, double v, double a) {
        m_x = x;
        m_v = v;
        m_a = a;
    }

    public Control100(double x, double v) {
        this(x, v, 0);
    }

    public Control100() {
        this(0, 0, 0);
    }

    /**
     * Return the model corresponding to this control, i.e. without acceleration.
     */
    public Model100 model() {
        return new Model100(m_x, m_v);
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

    public Control100 minus(Control100 other) {
        return new Control100(x() - other.x(), v() - other.v(), a() - other.a());
    }

    public Control100 plus(Control100 other) {
        return new Control100(x() + other.x(), v() + other.v(), a() + other.a());
    }

    public Control100 mult(double scaler) {
        return new Control100(m_x * scaler, m_v * scaler, m_a * scaler);
    }

    public boolean near(Control100 other, double tolerance) {
        return MathUtil.isNear(m_x, other.m_x, tolerance) &&
                MathUtil.isNear(m_v, other.m_v, tolerance);
    }

    @Override
    public String toString() {
        return String.format("Control100(X %5.3f V %5.3f A %5.3f)", m_x, m_v, m_a);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Control100) {
            Control100 rhs = (Control100) other;
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
