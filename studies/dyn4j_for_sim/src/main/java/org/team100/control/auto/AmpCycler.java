package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.util.Arg;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.IndexerSubsystem;

/**
 * Cycle from source to amp and back
 */
public class AmpCycler extends AutoPilot {
    private final DriveSubsystemInterface m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;

    public AmpCycler(
            DriveSubsystemInterface drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
    }

    @Override
    public boolean scoreAmp() {
        return enabled() && m_indexer.full();
    }

    @Override
    public boolean driveToSource() {
        return enabled() && !m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    @Override
    public boolean intake() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    @Override
    public boolean driveToNote() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }
}
