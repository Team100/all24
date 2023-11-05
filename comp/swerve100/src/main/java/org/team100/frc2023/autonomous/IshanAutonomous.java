package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;

public class IshanAutonomous extends Command {
    public static class Config {
        public double speedMetersPerSecond = 1;
        public double accelerationMetersPerSecondSquared = 1;
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;
    private final SwerveDriveKinematics m_kinematics;
    private final SwerveControllerCommand m_swerveController;

    public IshanAutonomous(
            SwerveDriveSubsystem robotDrive,
            SwerveDriveKinematics kinematics,
            RedundantGyro gyro) {
        m_robotDrive = robotDrive;
        m_kinematics = kinematics;
        Trajectory trajectory = genTrajectory();

        m_swerveController = new SwerveControllerCommand(
                m_robotDrive,
                trajectory,
                () -> new Rotation2d());
    }

    @Override
    public void initialize() {
        m_swerveController.initialize();
    }

    @Override
    public void execute() {
        m_swerveController.execute();
    }

    @Override
    public boolean isFinished() {
        return m_swerveController.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveController.end(interrupted);
    }

    private Trajectory genTrajectory() {
        TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
                m_config.speedMetersPerSecond,
                m_config.accelerationMetersPerSecondSquared)
                .setKinematics(m_kinematics);
        double controlPointAngle = Math.atan2(
                (1.071626 - m_robotDrive.getPose().getY()),
                (14.513558 - m_robotDrive.getPose().getX()));

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d(controlPointAngle)),
                List.of(
                        new Translation2d(
                                (14.513558 + m_robotDrive.getPose().getX()) / 2,
                                (1.071626 + m_robotDrive.getPose().getY()) / 2)),
                new Pose2d(14.513558, 1.071626, new Rotation2d(controlPointAngle)),
                kTrajectoryConfig);

        return exampleTrajectory;
    }
}
