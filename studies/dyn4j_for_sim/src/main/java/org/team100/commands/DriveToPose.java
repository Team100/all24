package org.team100.commands;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.util.Debug;
import org.team100.planner.DriveUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPose extends Command {
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final DriveSubsystemInterface m_drive;
    private final Supplier<Pose2d> m_goal;
    private final Supplier<Double> m_yBias;
    private final UnaryOperator<FieldRelativeVelocity> m_tactics;
    private final Tolerance m_tolerance;
    private final ForceViz m_viz;
    private final boolean m_debug;
    private final DriveUtil m_driveUtil;

    public DriveToPose(
            SwerveKinodynamics swerveKinodynamics,
            DriveSubsystemInterface drive,
            Supplier<Pose2d> goal,
            Supplier<Double> yBias,
            UnaryOperator<FieldRelativeVelocity> tactics,
            Tolerance tolerance,
            ForceViz viz,
            boolean debug) {
        m_swerveKinodynamics = swerveKinodynamics;
        m_drive = drive;
        m_goal = goal;
        m_yBias = yBias;
        m_tactics = tactics;
        m_tolerance = tolerance;
        m_viz = viz;
        m_debug = debug && Debug.enable();
        m_driveUtil = new DriveUtil(swerveKinodynamics, m_debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToPose");
        FieldRelativeVelocity desired = m_driveUtil.goToGoal(m_drive.getPose(), m_goal.get());
        // provide "lanes"
        desired = desired.plus(new FieldRelativeVelocity(0, m_yBias.get(), 0));
        if (m_debug)
            m_viz.desired(m_drive.getPose().getTranslation(), desired);
        if (m_debug)
            System.out.printf(" desired v %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(m_swerveKinodynamics.getMaxDriveVelocityM_S(), m_swerveKinodynamics.getMaxAngleSpeedRad_S());
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
