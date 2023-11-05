package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

/** Proxies RamseteCommand. */
public class Auton extends Command {
    private static final double kMaxSpeedMetersPerSecond = 1;
    private static final double kMaxAccelerationMetersPerSecondSquared = 1;
    private static final double kRamseteB = 2;
    private static final double kRamseteZeta = 0.7;
    private static final double kPDriveVel = 8.5;
    private final Command autonCommand;

    public Auton(DriveSubsystem robotDrive) {
        // Create a voltage constraint to ensure we don't accelerate too fast
        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                DriveSubsystem.feedforward,
                DriveSubsystem.kDriveKinematics,
                7);
        TrajectoryConfig config = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveSubsystem.kDriveKinematics)
                .addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at (1, 2) facing the +X direction
                new Pose2d(1, 2, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(2, 3), new Translation2d(3, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4, 2, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                robotDrive::getPose,
                new RamseteController(kRamseteB, kRamseteZeta),
                DriveSubsystem.feedforward,
                DriveSubsystem.kDriveKinematics,
                robotDrive::getWheelSpeeds,
                new PIDController(kPDriveVel, 0, 0),
                new PIDController(kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                robotDrive::tankDriveVolts,
                robotDrive);

        // Reset odometry to starting pose of trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        autonCommand = ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));

        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {
        autonCommand.initialize();
    }

    @Override
    public void execute() {
        autonCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        autonCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return autonCommand.isFinished();
    }
}
