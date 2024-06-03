package org.team100.control.auto;

import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pick up notes at the source and lob them to the scoring corner.
 * TODO: dedupe with speaker cycler
 */
public class Passer implements Autopilot {
    /** Ignore sightings further away than this. */
    private static final double kMaxNoteDistance = 8.0;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    
    private boolean m_enabled = false;

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
    public void begin() {
        m_enabled = true;
    }

    @Override
    public void reset() {
        m_enabled = false;
    }

    @Override
    public boolean pass() {
        return m_enabled && m_indexer.full();
    }

    @Override
    public boolean driveToSource() {
        return m_enabled && !noteNearby() && !m_indexer.full();
    }

    @Override
    public boolean intake() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    @Override
    public boolean driveToNote() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    @Override
    public void periodic() {
        //
    }

    /////////////////////////////////////////////////////

    private boolean noteNearby() {
        Pose2d pose = m_drive.getPose();
        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            return false;
        }
        return closestSighting.position().getDistance(pose.getTranslation()) <= kMaxNoteDistance;
    }

}
