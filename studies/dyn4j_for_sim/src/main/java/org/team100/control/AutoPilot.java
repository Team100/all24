package org.team100.control;

/** Autopilot implements enablement. */
public class AutoPilot implements Pilot {
    private boolean m_enabled = false;

    public boolean enabled() {
        return m_enabled;
    }

    /** Overrides should call super. :-( */
    @Override
    public void begin() {
        m_enabled = true;
    }

    /** Overrides should call super. :-( */
    @Override
    public void reset() {
        m_enabled = false;
    }
}
