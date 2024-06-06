package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pick up nearby notes and score them.
 * TODO: dedupe with speaker cycler.
 */
public class Scorer extends AutoPilot {
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
    private final Pose2d m_shooting;
    private State m_state;

    public Scorer(
            DriveSubsystem drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer,
            Pose2d shooting) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_shooting = shooting;
        m_state = State.Initial;
    }

    @Override
    public void begin() {
        super.begin();
        m_state = State.ToNoteForSpeaker;
    }

    @Override
    public void reset() {
        super.reset();
        m_state = State.Initial;
    }

    @Override
    public boolean scoreAmp() {
        return enabled() && m_state == State.ToAmp && m_indexer.full();
    }

    @Override
    public boolean scoreSpeaker() {
        return enabled() && m_state == State.ToSpeaker && m_indexer.full();
    }

    @Override
    public Pose2d shootingLocation() {
        return m_shooting;
    }

    @Override
    public boolean intake() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    @Override
    public boolean driveToNote() {
        return enabled() && m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
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

}
