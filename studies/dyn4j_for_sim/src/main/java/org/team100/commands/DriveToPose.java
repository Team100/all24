package org.team100.commands;

import java.util.function.Supplier;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.Drive;
import org.team100.sim.ForceViz;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private final DriveSubsystem m_drive;
    private final Supplier<Pose2d> m_goal;
    private final Supplier<Double> m_yBias;
    private final Tactics m_tactics;
    private final Tolerance m_tolerance;
    private final ForceViz m_viz;
    private final boolean m_debug;

    public DriveToPose(
            DriveSubsystem drive,
            Supplier<Pose2d> goal,
            Supplier<Double> yBias,
            Tactics tactics,
            Tolerance tolerance,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_goal = goal;
        m_yBias = yBias;
        m_tactics = tactics;
        m_tolerance = tolerance;
        m_viz = viz;
        m_debug = debug && Debug.enable();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToPose");
        FieldRelativeVelocity desired = Drive.goToGoal(m_drive.getPose(), m_goal.get(), m_debug);
        // provide "lanes"
        desired = desired.plus(new FieldRelativeVelocity(0, m_yBias.get(), 0));
        if (m_debug)
            m_viz.desired(m_drive.getPose(), desired);
        if (m_debug)
            System.out.printf(" desired v %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug)
            System.out.printf(" total v %s", v);
        m_drive.drive(v);
    }

    @Override
    public boolean isFinished() {
        FieldRelativeDelta t = FieldRelativeDelta.delta(m_drive.getPose(), m_goal.get());
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        double velocity = m_drive.getVelocity().norm();
        if (m_debug)
            System.out.printf("translation error %5.2f rotation error %5.2f\n",
                    translationError, rotationError);
        return translationError < m_tolerance.kTranslationTolerance()
                && Math.abs(rotationError) < m_tolerance.kAngularTolerance()
                && velocity < m_tolerance.kVelocityTolerance();
    }
}
