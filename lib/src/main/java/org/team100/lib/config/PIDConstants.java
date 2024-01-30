package org.team100.lib.config;

/** Uses digital inputs 4 and 5. */
public class PIDConstants {
    private final double m_p;
    private final double m_i;
    private final double m_d;
    private final double m_iZone;

    public PIDConstants(double p) {
       m_p = p;
       m_i = 0;
       m_d = 0;
       m_iZone = 0;
    }

    public PIDConstants(double p, double i, double d) {
       m_p = p;
       m_i = i;
       m_d = d;
       m_iZone = 0;
    }

    public PIDConstants(double p, double i, double d, double iZone) {
       m_p = p;
       m_i = i;
       m_d = d;
       m_iZone = iZone;
    }

    public double getP(){
        return m_p;
    }

    public double getI(){
        return m_i;
    }

    public double getD(){
        return m_d;
    }

    public double getIZone(){
        return m_iZone;
    }
}
