package org.team100.commands;

import java.util.Optional;

import org.team100.Debug;
import org.team100.control.Pilot;
import org.team100.field.StagedNote;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.Drive;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive to a staged note location */
public class GoToStaged extends Command {
    private final Pilot m_pilot;
    private final IndexerSubsystem m_indexer;
    private final DriveSubsystem m_drive;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public GoToStaged(
            Pilot pilot,
            IndexerSubsystem indexer,
            DriveSubsystem drive,
            CameraSubsystem camera,
            Tactics tactics,
            boolean debug) {
        Arg.nonnull(pilot);
        Arg.nonnull(indexer);
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_pilot = pilot;
        m_indexer = indexer;
        m_drive = drive;
        m_debug = debug && Debug.enable();
        m_tactics = tactics;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("GoToStaged");
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        int goalNoteId = m_pilot.goalNote();
        if (goalNoteId == 0)
            return;
        Optional<StagedNote> n = StagedNote.get(goalNoteId);
        if (n.isEmpty())
            return;

        FieldRelativeVelocity desired = Drive.goToGoalAligned(
                m_tactics,
                m_indexer,
                pose,
                n.get().getLocation(),
                m_debug);
        m_drive.drive(desired);
    }
}
