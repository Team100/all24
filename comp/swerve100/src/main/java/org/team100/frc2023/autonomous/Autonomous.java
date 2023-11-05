package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.DriveMobility;
import org.team100.frc2023.commands.arm.ArmTrajectory;
import org.team100.frc2023.commands.arm.SetCubeMode;
import org.team100.frc2023.commands.manipulator.Intake;
import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmInterface;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.autonomous.DriveStop;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.RedundantGyroInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autonomous extends SequentialCommandGroup {
    public static class Config {
        public double kArmExtendTimeout = 1.5;
        public double kManipulatorRunTimeout = 0.2;
        public double kArmSafeTimeout = 2;
        public double kStopTimeout = 1;        
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;
    private final FrameTransform m_transform;
    private final ArmInterface m_arm;
    private final ManipulatorInterface m_manipulator;
    private final RedundantGyroInterface m_gyro;
    private final LEDIndicator m_indicator;

    // TODO: make routine an enum
    public Autonomous(
            SwerveDriveSubsystem robotDrive,
            FrameTransform transform,
            ArmInterface arm,
            ManipulatorInterface manipulator,
            RedundantGyroInterface gyro,
            LEDIndicator indicator,
            int routine) {
        m_robotDrive = robotDrive;
        m_transform = transform;
        m_arm = arm;
        m_manipulator = manipulator;
        m_gyro = gyro;
        m_indicator = indicator;

        if (routine == 0) {
            placeCube();

        } else if (routine == 1) {
            placeCube();
            autoLevel(false);
            
        } else if (routine == 2) {
            placeCube();
            driveOutAndBack();
            autoLevel(true);
        }
    }

    private void placeCube() {
        addCommands(
                new SetCubeMode(m_arm, m_indicator),
                timeout(new ArmTrajectory(ArmPosition.HIGH, m_arm, false), m_config.kArmExtendTimeout),
                timeout(new Intake(m_manipulator), m_config.kManipulatorRunTimeout),
                timeout(new ArmTrajectory(ArmPosition.SAFE, m_arm, false), m_config.kArmSafeTimeout),
                new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)));
    }

    private void driveOutAndBack() {
        addCommands(
                new DriveMobility(m_robotDrive),
                timeout(new DriveStop(m_robotDrive), m_config.kStopTimeout),
                new DriveToThreshold(m_robotDrive));
    }

    private void autoLevel(boolean reversed) {
        addCommands(
                new AutoLevel(reversed, m_robotDrive, m_gyro, m_transform));
    }

    // TODO: why do we need a timeout?
    private Command timeout(Command command, double seconds) {
        return new ParallelDeadlineGroup(new WaitCommand(seconds), command);
    }
}
