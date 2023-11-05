package org.team100.frc2023.commands;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveMobility extends Command {
    public static class Config {
        public double kCommunitySizeMeters = 5.8;
        public double kXSpeedM_S = 1.5;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;

    private boolean done;

    /** Drive forward, exiting the community area. */
    public DriveMobility(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();
        // TODO: replace this with a waypoint
        if (currentPose.getX() < m_config.kCommunitySizeMeters) {

            Twist2d fieldRelative = new Twist2d(m_config.kXSpeedM_S, 0, 0);
            
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
