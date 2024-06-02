package org.team100.control.auto;

import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Alternates between speaker and amp.
 * 
 * TODO: add alliance input for choosing
 * TODO: notice the amplified mode
 */
public class AlternatingCycler implements Autopilot {
    /** Ignore sightings further away than this. */
    private static final double kMaxNoteDistance = 8.0;

    private enum State {
        Initial,
        ToSource,
        ToAmp,
        ToSpeaker
    }

    private State m_state;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;

    private boolean m_enabled = false;

    // placeholder for alliance strategy input or amplification input
    private boolean ampNext = false;

    public AlternatingCycler(
            DriveSubsystem drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer) {
        m_state = State.Initial;

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
        m_state = State.ToSource;
    }

    @Override
    public void reset() {
        m_enabled = false;
        m_state = State.Initial;
    }

    /**
     * the autopilot makes this a sequence that includes getting rid of the note, so
     * the indexer fullness is a completion indicator.
     * 
     * TODO: do the sequence here?
     */
    @Override
    public boolean driveToAmp() {
        return m_enabled && m_state == State.ToAmp && m_indexer.full();
    }

    @Override
    public boolean driveToSpeaker() {
        return m_enabled && m_state == State.ToSpeaker && m_indexer.full();
    }

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
        if (m_state == State.ToSource && m_indexer.full()) {
            // we just picked
            if (ampNext) {
                m_state = State.ToAmp;
            } else {
                m_state = State.ToSpeaker;
            }
            ampNext = !ampNext;
        } else if (m_state != State.ToSource && !m_indexer.full()) {
            // we just scored.
            m_state = State.ToSource;
        }
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
