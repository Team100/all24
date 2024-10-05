package org.team100.lib.pilot;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;

import org.team100.lib.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * This is an autopilot for the 2024 game.
 * 
 * Hang around the corner.
 * 
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

    private final Supplier<Pose2d> m_drive;
    private final Predicate<Pose2d> m_camera;
    private final BooleanSupplier m_indexer;
    private final BooleanSupplier m_amplified;
    private final Pose2d m_shooting;
    private final Pose2d m_corner;
    private State m_state;

    /**
     * Note the use of java functional classes to decouple from the simulator.
     * 
     * @param drive     the current robot pose
     * @param camera    true if there is a note near the supplied pose
     * @param indexer   true if the indexer is full
     * @param amplified true if the speaker is amplified
     * @param shooting  a configuration, red is different than blue
     * @param corner    a configuration, red is different than blue
     */
    public Scorer(
            Supplier<Pose2d> drive,
            Predicate<Pose2d> camera,
            BooleanSupplier indexer,
            BooleanSupplier amplified,
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
        return enabled() && !m_amplified.getAsBoolean() && m_indexer.getAsBoolean();
    }

    @Override
    public boolean scoreSpeaker() {
        // return enabled() && m_state == State.ToSpeaker && m_indexer.full();
        return enabled() && m_amplified.getAsBoolean() && m_indexer.getAsBoolean();
    }

    @Override
    public Pose2d shootingLocation() {
        return m_shooting;
    }

    @Override
    public boolean intake() {
        return enabled() && m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
    }

    @Override
    public boolean driveToNote() {
        return enabled() && m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
    }

    @Override
    public boolean driveToCorner() {
        return enabled() && !m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
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
                if (m_indexer.getAsBoolean())
                    m_state = State.ToSpeaker;
                break;
            case ToNoteForAmp:
                if (m_indexer.getAsBoolean())
                    m_state = State.ToAmp;
                break;
            case ToSpeaker:
                if (!m_indexer.getAsBoolean())
                    m_state = State.ToNoteForAmp;
                break;
            case ToAmp:
                if (!m_indexer.getAsBoolean())
                    m_state = State.ToNoteForSpeaker;
                break;
        }

    }
}
