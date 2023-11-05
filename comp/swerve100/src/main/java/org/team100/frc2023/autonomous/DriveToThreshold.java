package org.team100.frc2023.autonomous;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToThreshold extends Command {
    public static class Config {
        public double kEdgeOfRampMeters = 4.1;
        public double kXSpeedM_S = -2.0;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;

    private boolean done;

    /** Drive back to the edge of the charge station. */
    public DriveToThreshold(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (m_robotDrive.getPose().getX() > m_config.kEdgeOfRampMeters) {
            Twist2d fieldRelative = new Twist2d(m_config.kXSpeedM_S, 0, 0);

            Pose2d currentPose = m_robotDrive.getPose();
            SwerveState manualState = SwerveDriveSubsystem.incremental(currentPose, fieldRelative);
            m_robotDrive.setDesiredState(manualState);
        } else {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.truncate();
    }
}
