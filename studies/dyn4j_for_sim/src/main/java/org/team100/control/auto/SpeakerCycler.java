package org.team100.control.auto;

import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Cycle from source to speaker and back.
 * 
 * Watches for nearby notes, picks them up opportunistically.
 * 
 * Mode depends on indexer fullness.
 * 
 * Since this observes a subsystem, it needs to be constructed after the
 * subsystem is constructed.
 */
public class SpeakerCycler implements Autopilot {
    /** Ignore sightings further away than this. */
    private static final double kMaxNoteDistance = 8.0;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;

    private boolean m_enabled = false;

    // todo: make these into observers not subsystems.
    public SpeakerCycler(
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

    // drive to the speaker if there's a note in the indexer.
    @Override
    public boolean scoreSpeaker() {
        return m_enabled && m_indexer.full();
    }

    // drive to the source if there's no note nearby and no note in the indexer.
    @Override
    public boolean driveToSource() {
        return m_enabled && !noteNearby() && !m_indexer.full();
    }

    // intake if there's a note nearby and none in the indexer.
    @Override
    public boolean intake() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    // drive to the note if there's one nearby and no note in the indexer.
    @Override
    public boolean driveToNote() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    @Override
    public void periodic() {
        //
    }

    ///////////////////////////////////////////////////////////////////

    private boolean noteNearby() {
        Pose2d pose = m_drive.getPose();
        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            return false;
        }
        return closestSighting.position().getDistance(pose.getTranslation()) <= kMaxNoteDistance;
    }

}
