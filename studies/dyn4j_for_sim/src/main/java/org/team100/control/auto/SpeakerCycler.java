package org.team100.control.auto;

import org.team100.control.AutoPilot;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.util.Arg;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.IndexerSubsystem;

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
    private final DriveSubsystemInterface m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    private final Pose2d m_shooting;

    // todo: make these into observers not subsystems.
    public SpeakerCycler(
            DriveSubsystemInterface drive,
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
    }

    // drive to the speaker if there's a note in the indexer.
    @Override
    public boolean scoreSpeaker() {
        return enabled() && m_indexer.full();
    }

    // drive to the source if there's no note nearby and no note in the indexer.
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
    public Pose2d shootingLocation() {
        return m_shooting;
    }
}
