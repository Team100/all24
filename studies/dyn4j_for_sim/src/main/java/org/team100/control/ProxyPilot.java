package org.team100.control;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

/**
 * Proxy allows the actual pilot to be changed on the fly.
 */
public class ProxyPilot implements Pilot {
    private Pilot m_pilot;

    public ProxyPilot(Pilot initial) {
        m_pilot = initial;
    }

    public void setPilot(Pilot pilot) {
        m_pilot = pilot;
    }

    @Override
    public FieldRelativeVelocity driveVelocity() {
        return m_pilot.driveVelocity();
    }

    @Override
    public boolean intake() {
        return m_pilot.intake();
    }

    @Override
    public boolean outtake() {
        return m_pilot.outtake();
    }

    @Override
    public boolean shoot() {
        return m_pilot.shoot();
    }

    @Override
    public boolean lob() {
        return m_pilot.lob();
    }

    @Override
    public boolean amp() {
        return m_pilot.amp();
    }

    @Override
    public boolean rotateToShoot() {
        return m_pilot.rotateToShoot();
    }

    @Override
    public boolean driveToSpeaker() {
        return m_pilot.driveToSpeaker();
    }

    @Override
    public void onEnd() {
        m_pilot.onEnd();
    }

    @Override
    public void begin() {
        m_pilot.begin();
    }

    @Override
    public void reset() {
        m_pilot.reset();
    }

}
