package org.team100.lib.telemetry;

import edu.wpi.first.wpilibj.Timer;

public class LoadShedder {
    private final Timer m_timer;
    private final double m_limitS;

    public LoadShedder(double limitS) {
        m_timer = new Timer();
        m_limitS = limitS;
    }

    public void start() {
        // this is called in robotPeriodic at the start of each cycle.
        m_timer.restart();
    }

    public boolean expired() {
        return m_timer.hasElapsed(m_limitS);
    }

}
