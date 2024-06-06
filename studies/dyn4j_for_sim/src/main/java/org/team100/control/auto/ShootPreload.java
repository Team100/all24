package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

public class ShootPreload extends AutoPilot {

    private final DriveSubsystem m_drive;

    public ShootPreload(DriveSubsystem drive) {
        m_drive = drive;
    }

    /** Shoot from wherever you are. */
    @Override
    public Pose2d shootingLocation() {
        return m_drive.getPose();
    }

    @Override
    public boolean scoreSpeaker() {
        return enabled();
    }

}
