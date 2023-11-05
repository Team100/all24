package org.team100.frc2023.autonomous;

import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class DriveToThreshold extends Command {
    public static class Config {
        public double kEdgeOfRampMeters = 4.1;
        public double kXSpeedM_S = -2.0;
    }

    private final Config m_config = new Config();
    private final Drivetrain m_robotDrive;

    private boolean done;

    /** Drive back to the edge of the charge station. */
    public DriveToThreshold(Drivetrain robotDrive) {
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
            SwerveState manualState = Drivetrain.incremental(currentPose, fieldRelative);
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