package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToAprilTag extends Command {
    public static class Config {
        public double speedMetersPerSecond = 2;
        public double accelerationMetersPerSecondSquared = 1;
    }

    private final Config m_config = new Config();

    private final SwerveControllerCommand m_swerveController;

    public MoveToAprilTag(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            int tagID,
            RedundantGyro gyro) {
        Trajectory trajectory = genTrajectory(m_robotDrive, kinematics, layout, tagID);
        m_swerveController = new SwerveControllerCommand(
                m_robotDrive,
                trajectory,
                () -> new Rotation2d());
        addRequirements(m_robotDrive);
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

    private Trajectory genTrajectory(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            int tagID) {

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                m_config.speedMetersPerSecond,
                m_config.accelerationMetersPerSecondSquared)
                .setKinematics(kinematics);

        Pose2d aprilPose = layout.getTagPose(tagID).get().toPose2d();

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                m_robotDrive.getPose(),
                List.of(),
                new Pose2d(
                        aprilPose.getX() - 1,
                        aprilPose.getY(),
                        new Rotation2d(aprilPose.getRotation().getDegrees())),
                trajectoryConfig);

        return exampleTrajectory;
    }
}
