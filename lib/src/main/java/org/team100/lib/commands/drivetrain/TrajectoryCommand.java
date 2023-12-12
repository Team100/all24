package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follow a trajectory.
 */
public class TrajectoryCommand extends Command {
    private final Telemetry t = Telemetry.get();
    private final Trajectory m_trajectory;
    private final SwerveDriveSubsystemInterface m_swerve;
    private final Timer m_timer;
    private final HolonomicDriveController3 m_controller;

    public TrajectoryCommand(
            Trajectory trajectory,
            SwerveDriveSubsystemInterface swerve,
            HolonomicDriveController3 controller) {
        m_trajectory = trajectory;
        m_swerve = swerve;
        m_controller = controller;
        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.stop();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);
        Pose2d currentPose = m_swerve.getPose();
        // TODO: rotation profile, use new trajectory type.
        State lastState = m_trajectory.sample(Double.MAX_VALUE);
        SwerveState reference = SwerveState.fromState(desiredState, lastState.poseMeters.getRotation());
        Twist2d fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        m_swerve.driveInFieldCoords(fieldRelativeTarget);

        t.log(Level.DEBUG, "/TrajectoryCommand/Desired X", desiredState.poseMeters.getX());
        t.log(Level.DEBUG, "/TrajectoryCommand/Desired Y", desiredState.poseMeters.getY());
        t.log(Level.DEBUG, "/TrajectoryCommand/Pose X", m_swerve.getPose().getX());
        t.log(Level.DEBUG, "/TrajectoryCommand/Pose Y", m_swerve.getPose().getY());
        t.log(Level.DEBUG, "/TrajectoryCommand/Desired Rot", lastState.poseMeters.getRotation().getRadians());
        t.log(Level.DEBUG, "/TrajectoryCommand/Pose Rot", m_swerve.getPose().getRotation().getRadians());
        t.log(Level.DEBUG, "/TrajectoryCommand/Time", curTime);
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
    }

}
