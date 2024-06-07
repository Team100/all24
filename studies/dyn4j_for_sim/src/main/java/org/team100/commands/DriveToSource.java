package org.team100.commands;

import java.util.function.Supplier;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.Drive;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command should be followed by DriveToNote, so that the goal of this
 * command can be very approximate.
 */
public class DriveToSource extends Command {
    private final DriveSubsystem m_drive;
    private final Supplier<Pose2d> m_goal;
    private final Tolerance m_tolerance;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToSource(
            DriveSubsystem drive,
            CameraSubsystem camera,
            Supplier<Pose2d> goal,
            Tactics tactics,
            Tolerance tolerance,
            boolean debug) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_drive = drive;
        m_goal = goal;
        m_tolerance = tolerance;
        m_debug = debug && Debug.enable();
        m_tactics = tactics;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToSource");
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        FieldRelativeVelocity desired = Drive.goToGoal(pose, m_goal.get(), m_debug);
        if (m_debug)
            ForceViz.put("desired", pose, desired);
        if (m_debug)
            System.out.printf(" desired %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired);
        if (m_debug)
            System.out.printf(" tactics %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug)
            System.out.printf(" final %s\n", v);
        m_drive.drive(v);
    }

    /** TODO: i think this never matters, the pilot cancels the command */
    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_goal.get());
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        double velocity = m_drive.getVelocity().norm();
        return translationError < m_tolerance.kTranslationTolerance()
                && Math.abs(rotationError) < m_tolerance.kAngularTolerance()
                && velocity < m_tolerance.kVelocityTolerance();
    }

}
