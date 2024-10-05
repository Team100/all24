package org.team100.lib.commands.semiauto;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.planner.DriveUtil;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.util.Arg;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command should be followed by DriveToNote, so that the goal of this
 * command can be very approximate.
 * 
 * Note this command never ends, so the pilot will need to notice the robot's
 * position and decide to do something else.
 */
public class DriveToSource extends Command {
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final DriveSubsystemInterface m_drive;
    private final Supplier<Pose2d> m_goal;
    private final Supplier<Double> m_yBias;
    private final boolean m_debug;
    private final UnaryOperator<FieldRelativeVelocity> m_tactics;
    private final ForceViz m_viz;
    private final DriveUtil m_driveUtil;

    public DriveToSource(
            SwerveKinodynamics swerveKinodynamics,
            DriveSubsystemInterface drive,
            Supplier<Pose2d> goal,
            Supplier<Double> yBias,
            UnaryOperator<FieldRelativeVelocity> tactics,
            ForceViz viz,
            boolean debug) {
        Arg.nonnull(drive);
        m_swerveKinodynamics = swerveKinodynamics;
        m_drive = drive;
        m_goal = goal;
        m_yBias = yBias;
        m_debug = debug && Debug.enable();
        m_tactics = tactics;
        m_viz = viz;
        m_driveUtil = new DriveUtil(swerveKinodynamics, drive, viz, tactics, m_debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToSource");
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        FieldRelativeVelocity desired = m_driveUtil.goToGoal(pose, m_goal.get());
        // provide "lanes"
        desired = desired.plus(new FieldRelativeVelocity(0, m_yBias.get(), 0));
        if (m_debug)
            m_viz.desired(pose.getTranslation(), desired);
        if (m_debug)
            System.out.printf(" desired %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired);
        if (m_debug)
            System.out.printf(" tactics %s", v);
        v = v.plus(desired);
        v = v.clamp(m_swerveKinodynamics.getMaxDriveVelocityM_S(), m_swerveKinodynamics.getMaxAngleSpeedRad_S());
        if (m_debug)
            System.out.printf(" final %s\n", v);
        m_drive.drive(v);
    }
}
