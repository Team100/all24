package org.team100.frc2023.commands;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

// TODO: discard this class, use a holonomic trajectory instead, e.g. like PathPlanner Waypoints
public class SwerveControllerCommand {
    private final SwerveDriveSubsystem m_robotDrive;
    private final Timer m_timer;
    private final Trajectory m_trajectory;
    private final Supplier<Rotation2d> m_desiredRotation;

    public SwerveControllerCommand(
            SwerveDriveSubsystem robotDrive,
            Trajectory trajectory,
            Supplier<Rotation2d> rotationSupplier) {
        m_robotDrive = robotDrive;
        m_trajectory = trajectory;
        m_desiredRotation = rotationSupplier;
        m_timer = new Timer();
    }

    public void initialize() {
        m_timer.restart();
    }

    public void execute() {
        State desiredState = m_trajectory.sample(m_timer.get());
        Rotation2d desiredHeading = m_desiredRotation.get();

        SwerveState desiredSwerveState = SwerveState.fromState(desiredState,desiredHeading);

        m_robotDrive.setDesiredState(desiredSwerveState);
    }

    public void end(boolean interrupted) {
        m_timer.stop();
    }

    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
