
package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.DriveMobility;
import org.team100.lib.autonomous.DriveStop;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Drivetrain;

public class Autonomous extends SequentialCommandGroup {
    public static class Config {
        public double kArmExtendTimeout = 2;
        public double kManipulatorRunTimeout = 0.2;
        public double kArmSafeTimeout = 2;
        public double kStopTimeout = 1;
    }

    private final Config m_config = new Config();
    private final Drivetrain m_robotDrive;
    private final FrameTransform m_transform;
    // private final ArmInterface m_arm;
    // private final ManipulatorInterface m_manipulator;
    private final AnalogGyro m_gyro;
    // private final LEDIndicator m_indicator;

    // TODO: make routine an enum
    public Autonomous(
            Drivetrain robotDrive,
            FrameTransform transform,
            // ArmInterface arm,
            // ManipulatorInterface manipulator,
            AnalogGyro gyro,
            // LEDIndicator indicator,
            int routine) {
        m_robotDrive = robotDrive;
        m_transform = transform;
        // m_arm = arm;
        // m_manipulator = manipulator;
        m_gyro = gyro;
        // m_indicator = indicator;
        // addRequirements(m_robotDrive);

        if (routine == 0) {
            placeCube();

        } else if (routine == 1) {
            // new WaitCommand(1);
            // new ResetPose(robotDrive, routine, routine, routine)

            // reset180();
            placeCube();
            autoLevel(false);

        } else if (routine == 2) {
            placeCube();
            driveOutAndBack();
            autoLevel(true);
        }
        // addCommands(
        // // new ResetPose(m_robotDrive, 0, 0, Math.PI),
        // // new DeadDrivetrain(m_robotDrive),
        // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
        // // new ResetPose(m_robotDrive, 0, 0, Math.PI),
        // new DeadDrivetrain(m_robotDrive),

        // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new Intake(/*
        // m_manipulator */))

        // timeout(new ArmTrajectory(ArmPosition.AUTO, m_arm, false),
        // m_config.kArmExtendTimeout),
        // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
        // ArmTrajectory(ArmPosition.AUTO, m_arm, false))
        // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
        // Intake(m_manipulator)),
        // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
        // ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
        // new AutoLevel(false, m_robotDrive, m_gyro, m_transform)
        // );
    }

    private void placeCube() {
        addCommands(
                // new ResetPose(m_robotDrive, 0, 0, Math.PI),
                new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                // new ResetPose(m_robotDrive, 0, 0, Math.PI),

                // timeout(new ArmTrajectory(ArmPosition.AUTO, m_arm, false),
                // m_config.kArmExtendTimeout),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
                // ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
                // Intake(m_manipulator)),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new
                // ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
                // new AutoLevel(false, m_robotDrive, m_gyro, m_transform));
                // timeout(new Intake(m_manipulator), m_config.kManipulatorRunTimeout),
                // timeout(new ArmTrajectory(ArmPosition.SAFE, m_arm, false),
                // m_config.kArmSafeTimeout));
                new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)));

    }

    private void reset180() {
        addCommands(
                // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                new ResetPose(m_robotDrive, 0, 0, Math.PI));

    }

    private void driveOutAndBack() {
        addCommands(
                new DriveMobility(m_robotDrive),
                timeout(new DriveStop(m_robotDrive), m_config.kStopTimeout),
                new DriveToThreshold(m_robotDrive));
    }

    private void autoLevel(boolean reversed) {
        addCommands(
                // new ResetPose(m_robotDrive, 0, 0, 0),
                // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                new AutoLevel(reversed, m_robotDrive, m_gyro, m_transform));
    }

    // TODO: why do we need a timeout?
    private Command timeout(Command command, double seconds) {
        return new ParallelDeadlineGroup(new WaitCommand(seconds), command);
    }
}
