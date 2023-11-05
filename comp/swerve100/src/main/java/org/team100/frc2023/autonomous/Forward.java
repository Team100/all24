package org.team100.frc2023.autonomous;

import java.util.List;

import org.team100.frc2023.commands.SwerveControllerCommand;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;

// TODO: do we need this?
public class Forward extends Command {
    private final SwerveControllerCommand m_swerveController;

    public Forward(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            double x,
            RedundantGyro gyro) {
        Trajectory trajectory = genTrajectory(m_robotDrive, kinematics, x);
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

    private static Trajectory genTrajectory(
            SwerveDriveSubsystem m_robotDrive,
            SwerveDriveKinematics kinematics,
            double x) {
        Pose2d currentRobotPose = m_robotDrive.getPose();
        double xRobot = currentRobotPose.getX();
        double yRobot = currentRobotPose.getY();
        Rotation2d rotRobot = currentRobotPose.getRotation();
        if (x < 0) {
            rotRobot = new Rotation2d(rotRobot.getRadians() - Math.PI);
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(4, 3).setKinematics(kinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(xRobot, yRobot, rotRobot),
                List.of(),
                new Pose2d(xRobot + x, yRobot, rotRobot),
                trajectoryConfig);
        return exampleTrajectory;
    }

}
