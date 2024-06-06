package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

/**
 * Alternates between speaker and amp.
 * 
 * TODO: add alliance input for choosing
 * TODO: notice the amplified mode
 */
public class AlternatingCycler extends AutoPilot {
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
        super.begin();
        m_state = State.ToSource;
    }

    @Override
    public void reset() {
        super.reset();
        m_state = State.Initial;
    }

    /**
     * the autopilot makes this a sequence that includes getting rid of the note, so
     * the indexer fullness is a completion indicator.
     * 
     * TODO: do the sequence here?
     */
    @Override
    public boolean scoreAmp() {
        return enabled() && m_state == State.ToAmp && m_indexer.full();
    }

    @Override
    public boolean scoreSpeaker() {
        return enabled() && m_state == State.ToSpeaker && m_indexer.full();
    }

    @Override
    public boolean driveToSource() {
        return enabled() && !m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    // intake if there's a note nearby and none in the indexer.
    @Override
    public boolean intake() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    // drive to the note if there's one nearby and no note in the indexer.
    @Override
    public boolean driveToNote() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
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

}
