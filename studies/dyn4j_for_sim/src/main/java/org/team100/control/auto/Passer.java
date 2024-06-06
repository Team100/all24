package org.team100.control.auto;

import org.team100.control.AutoPilot;
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
public class Passer extends AutoPilot {
    /** Ignore sightings further away than this. */
    private static final double kMaxNoteDistance = 8.0;

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
        return enabled() && !noteNearby() && !m_indexer.full();
    }

    @Override
    public boolean intake() {
        return enabled() && noteNearby() && !m_indexer.full();
    }

    @Override
    public boolean driveToNote() {
        return enabled() && noteNearby() && !m_indexer.full();
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
