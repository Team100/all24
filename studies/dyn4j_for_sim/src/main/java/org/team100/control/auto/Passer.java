package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

/**
 * Pick up notes at the source and lob them to the scoring corner.
 * TODO: dedupe with speaker cycler
 */
public class Passer extends AutoPilot {
    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;

    public Passer(
            DriveSubsystem drive,
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
    public boolean pass() {
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
