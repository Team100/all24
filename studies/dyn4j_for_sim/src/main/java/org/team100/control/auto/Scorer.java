package org.team100.control.auto;

import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pick up nearby notes and score them.
 * TODO: dedupe with speaker cycler.
 */
public class Scorer implements Autopilot {
    /** Ignore sightings further away than this. */
    private static final double kMaxNoteDistance = 8.0;

    private enum State {
        Initial,
        ToNoteForSpeaker,
        ToNoteForAmp,
        ToSpeaker,
        ToAmp
    }

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    private State m_state;
    private boolean m_enabled = false;

    public Scorer(
            DriveSubsystem drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_state = State.Initial;
    }

    @Override
    public void begin() {
        m_enabled = true;
        m_state = State.ToNoteForSpeaker;
    }

    @Override
    public void reset() {
        m_enabled = false;
        m_state = State.Initial;
    }

    @Override
    public boolean scoreAmp() {
        return m_enabled && m_state == State.ToAmp && m_indexer.full();
    }

    @Override
    public boolean scoreSpeaker() {
        return m_enabled && m_state == State.ToSpeaker && m_indexer.full();
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
        switch (m_state) {
            case Initial:
                // ignore
                break;
            case ToNoteForSpeaker:
                if (m_indexer.full())
                    m_state = State.ToSpeaker;
                break;
            case ToNoteForAmp:
                if (m_indexer.full())
                    m_state = State.ToAmp;
                break;
            case ToSpeaker:
                if (!m_indexer.full())
                    m_state = State.ToNoteForAmp;
                break;
            case ToAmp:
                if (!m_indexer.full())
                    m_state = State.ToNoteForSpeaker;
                break;
        }

    }

    //////////////////////////////////////////////////

    private boolean noteNearby() {
        Pose2d pose = m_drive.getPose();
        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            return false;
        }
        return closestSighting.position().getDistance(pose.getTranslation()) <= kMaxNoteDistance;
    }
}
