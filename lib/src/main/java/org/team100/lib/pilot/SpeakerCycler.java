package org.team100.lib.pilot;

import java.util.function.BooleanSupplier;
import java.util.function.Predicate;
import java.util.function.Supplier;

import org.team100.lib.util.Arg;

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
public class SpeakerCycler extends AutoPilot {
    private final Supplier<Pose2d> m_drive;
    private final Predicate<Pose2d> m_camera;
    private final BooleanSupplier m_indexer;
    private final Pose2d m_shooting;

    /**
     * 
     * @param drive    the current robot pose
     * @param camera   true if there's a note nearby the supplied pose.
     * @param indexer  true if the indexes is full
     * @param shooting configuration, red and blue are different.
     */
    public SpeakerCycler(
            Supplier<Pose2d> drive,
            Predicate<Pose2d> camera,
            BooleanSupplier indexer,
            Pose2d shooting) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_shooting = shooting;
    }

    // drive to the speaker if there's a note in the indexer.
    @Override
    public boolean scoreSpeaker() {
        return enabled() && m_indexer.getAsBoolean();
    }

    // drive to the source if there's no note nearby and no note in the indexer.
    @Override
    public boolean driveToSource() {
        return enabled() && !m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
    }

    // intake if there's a note nearby and none in the indexer.
    @Override
    public boolean intake() {
        return enabled() && m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
    }

    // drive to the note if there's one nearby and no note in the indexer.
    @Override
    public boolean driveToNote() {
        return enabled() && m_camera.test(m_drive.get()) && !m_indexer.getAsBoolean();
    }

    @Override
    public Pose2d shootingLocation() {
        return m_shooting;
    }
}
