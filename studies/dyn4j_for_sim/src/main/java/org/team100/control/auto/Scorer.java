package org.team100.control.auto;

import java.util.function.Supplier;

import org.team100.control.AutoPilot;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.util.Arg;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pick up nearby notes and score them.
 * 
 * if not amplified, score in the amp.
 * if amplified, score in the speaker.
 */
public class Scorer extends AutoPilot {
    private enum State {
        Initial,
        ToNoteForSpeaker,
        ToNoteForAmp,
        ToSpeaker,
        ToAmp
    }

    private final DriveSubsystemInterface m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    private final Supplier<Boolean> m_amplified;
    private final Pose2d m_shooting;
    private final Pose2d m_corner;
    private State m_state;

    public Scorer(
            DriveSubsystemInterface drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer,
            Supplier<Boolean> amplified,
            Pose2d shooting,
            Pose2d corner) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_amplified = amplified;
        m_shooting = shooting;
        m_corner = corner;
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
        // return enabled() && m_state == State.ToAmp && m_indexer.full();
        return enabled() && !m_amplified.get() && m_indexer.full();
    }

    @Override
    public boolean scoreSpeaker() {
        // return enabled() && m_state == State.ToSpeaker && m_indexer.full();
        return enabled() && m_amplified.get() && m_indexer.full();
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
    public boolean driveToCorner() {
        return enabled() && !m_camera.noteNearby(m_drive.getPose()) && !m_indexer.full();
    }

    @Override
    public Pose2d cornerLocation() {
        return m_corner;
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
