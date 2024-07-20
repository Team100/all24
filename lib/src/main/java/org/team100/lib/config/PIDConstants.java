package org.team100.lib.config;

/**
 * Be careful of the units for these parameters: different units will be used by
 * different outboard motor controllers, and by the same outboard motor
 * controller when used in different ways. Please add a comment about units to
 * every use of this class.
 */
public class PIDConstants {
    private final double m_p;
    private final double m_i;
    private final double m_d;
    private final double m_iZone;

    public PIDConstants(double p) {
        this(p, 0, 0);
    }

    public PIDConstants(double p, double i, double d) {
        m_p = p;
        m_i = i;
        m_d = d;
        m_iZone = 0;
    }

    public double getP() {
        return m_p;
    }

    public double getI() {
        return m_i;
    }

    public double getD() {
        return m_d;
    }

    public double getIZone() {
        return m_iZone;
    }
}
