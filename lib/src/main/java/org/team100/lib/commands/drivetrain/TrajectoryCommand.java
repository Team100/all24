package org.team100.lib.commands.drivetrain;

import org.team100.lib.commands.Command100;
import org.team100.lib.controller.DriveMotionController;
import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.trajectory.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follow a WPI trajectory.
 * 
 * This uses the end state rotation as the setpoint for the entire trajectory,
 * which will put a lot of error into the controller.
 * 
 * The {@link DriveMotionController} classes are better, use them instead.
 */
public class TrajectoryCommand extends Command100 {
    private final Telemetry t = Telemetry.get();
    private final Trajectory m_trajectory;
    private final SwerveDriveSubsystem m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;

    public TrajectoryCommand(
            Trajectory trajectory,
            SwerveDriveSubsystem swerve,
            HolonomicDriveController3 controller) {
        m_trajectory = trajectory;
        m_swerve = swerve;
        m_controller = controller;
        m_timer = new Timer();
    }

    @Override
    public void initialize100() {
        m_controller.reset();
        m_timer.stop();
        m_timer.reset();
        m_timer.start();
        TrajectoryVisualization.setViz(m_trajectory);
    }

    @Override
    public void execute100(double dt) {
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        Pose2d currentPose = m_swerve.getPose();
        State lastState = m_trajectory.sample(Double.MAX_VALUE);
        SwerveState reference = SwerveState.fromState(desiredState, lastState.poseMeters.getRotation());
        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        m_swerve.driveInFieldCoords(fieldRelativeTarget, dt);

        t.log(Level.TRACE, m_name, "Desired X", desiredState.poseMeters.getX());
        t.log(Level.TRACE, m_name, "Desired Y", desiredState.poseMeters.getY());
        t.log(Level.TRACE, m_name, "Pose X", m_swerve.getPose().getX());
        t.log(Level.TRACE, m_name, "Pose Y", m_swerve.getPose().getY());
        t.log(Level.TRACE, m_name, "Desired Rot", lastState.poseMeters.getRotation().getRadians());
        t.log(Level.TRACE, m_name, "Pose Rot", m_swerve.getPose().getRotation().getRadians());
        t.log(Level.TRACE, m_name, "Time", curTime);
    }

    @Override
    public boolean isFinished() {
        if (m_trajectory == null)
            return true;
        return m_timer.get() > m_trajectory.getTotalTimeSeconds() && m_controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        TrajectoryVisualization.clear();
    }

}
