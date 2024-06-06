package org.team100.control.auto;

import org.team100.control.Pilot;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootPreload implements Pilot {

    private final DriveSubsystem m_drive;

    private boolean m_enabled = false;

    public ShootPreload(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public Pose2d shootingLocation() {
        // shoot from wherever you are
        return m_drive.getPose();
    }

    @Override
    public boolean scoreSpeaker() {
        return m_enabled;
    }

    @Override
    public void begin() {
        m_enabled = true;
    }

    @Override
    public void reset() {
        m_enabled = false;
    }

}
