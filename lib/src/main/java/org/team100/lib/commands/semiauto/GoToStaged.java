package org.team100.lib.commands.semiauto;

import java.util.Optional;

import org.team100.lib.field.StagedNote2024;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.pilot.Pilot;
import org.team100.lib.planner.DriveUtil;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.planner.Tactics;
import org.team100.lib.util.Arg;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive to a staged note location */
public class GoToStaged extends Command {
    /** Alignment tolerance for the intake */
    public static final double kIntakeAdmittanceRad = 0.2;

    private final Pilot m_pilot;
    private final DriveSubsystemInterface m_drive;
    private final boolean m_debug;
    private final DriveUtil m_driveUtil;

    public GoToStaged(
            SwerveKinodynamics swerveKinodynamics,
            Pilot pilot,
            DriveSubsystemInterface drive,
            Tactics tactics,
            ForceViz viz,
            boolean debug) {
        Arg.nonnull(pilot);
        Arg.nonnull(drive);
        m_pilot = pilot;
        m_drive = drive;
        m_debug = debug && Debug.enable();
        m_driveUtil = new DriveUtil(swerveKinodynamics, drive, viz, tactics, m_debug);
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
        Optional<StagedNote2024> n = StagedNote2024.get(goalNoteId);
        if (n.isEmpty())
            return;

        FieldRelativeVelocity desired = m_driveUtil.goToGoalAligned(
                kIntakeAdmittanceRad,
                pose,
                n.get().getLocation());
        m_drive.drive(desired);
    }
}
