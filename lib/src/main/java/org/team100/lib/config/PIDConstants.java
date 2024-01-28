package org.team100.lib.config;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

/** Uses digital inputs 4 and 5. */
public class PIDConstants {
    private final double m_p;
    private final double m_i;
    private final double m_d;
    private final double m_iZone;
    private final double m_period;


    public PIDConstants(double p, double i, double d) {
       m_p = p;
       m_i = i;
       m_d = d;
       m_iZone = 0;
       m_period = 0.02;
    }

    public PIDConstants(double p, double i, double d, double iZone, double period) {
       m_p = p;
       m_i = i;
       m_d = d;
       m_iZone = iZone;
       m_period = period;
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

    public double getPeriod(){
        return m_period;
    }

   

}
