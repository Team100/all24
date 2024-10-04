package org.team100.commands;

import java.util.Optional;

import org.team100.control.Pilot;
import org.team100.field.StagedNote;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.util.Arg;
import org.team100.lib.util.Debug;
import org.team100.planner.DriveUtil;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive to a staged note location */
public class GoToStaged extends Command {
    /** Alignment tolerance for the intake */
    public static final double kIntakeAdmittanceRad = 0.2;

    private final Pilot m_pilot;
    private final DriveSubsystemInterface m_drive;
    private final boolean m_debug;
    private final Tactics m_tactics;
    private final DriveUtil m_driveUtil;

    public GoToStaged(
            SwerveKinodynamics swerveKinodynamics,
            Pilot pilot,
            DriveSubsystem drive,
            CameraSubsystem camera,
            Tactics tactics,
            boolean debug) {
        Arg.nonnull(pilot);
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_pilot = pilot;
        m_drive = drive;
        m_debug = debug && Debug.enable();
        m_tactics = tactics;
        m_driveUtil = new DriveUtil(swerveKinodynamics, m_debug);
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

        FieldRelativeVelocity desired = m_driveUtil.goToGoalAligned(
                m_tactics,
                kIntakeAdmittanceRad,
                pose,
                n.get().getLocation());
        m_drive.drive(desired);
    }
}
