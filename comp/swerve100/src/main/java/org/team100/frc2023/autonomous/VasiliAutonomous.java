package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.arm.ArmTrajectory;
import org.team100.frc2023.commands.arm.SetCubeMode;
import org.team100.frc2023.commands.manipulator.Intake;
import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.frc2023.subsystems.arm.ArmSubsystem;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.RedundantGyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class VasiliAutonomous extends SequentialCommandGroup {
    ControlVectorList controlVectors = new ControlVectorList();

    public VasiliAutonomous(
            SwerveDriveSubsystem drive,
            FrameTransform chassisSpeedFactory,
            SwerveDriveKinematics kinematics, 
            RedundantGyro gyro,
            ArmSubsystem arm,
            ManipulatorInterface manipulator,
            LEDIndicator indicator) {

        controlVectors.add(new Spline.ControlVector(
                new double[] { drive.getPose().getX(), 0, 0 },
                new double[] { drive.getPose().getY(), 0, 0.0 }));

        controlVectors.add(new Spline.ControlVector(
                new double[] { 5.537, 0, 0 },
                new double[] { 4.922, 0, 0 }));

        controlVectors.add(new Spline.ControlVector(
                new double[] { 5.916, 0, 0 },
                new double[] { 2.656, 0, 0 }));

        addCommands(
                // TODO: do we need this?

                // moveFromStartingPoseToGamePiece
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // new Pose2d(
                // m_robotDrive.getPose().getX(),
                // m_robotDrive.getPose().getY(),
                // new Rotation2d(Math.PI / 2)),
                // new Pose2d(0, 0.92, new Rotation2d(Math.PI / 2)),
                // () -> new Rotation2d(Math.PI)),

                // new ResetRotation(m_robotDrive, new Rotation2d(Math.PI)),

                // VasiliWaypointTrajectory
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // controlVectors,
                // () -> new Rotation2d(Math.PI),
                // "output/GoToCube(x).wpilib.json"),

                // new WaitCommand(2),
                // new Rotate(m_robotDrive, 0),
                // new WaitCommand(2),
                // new Rotate(m_robotDrive, Math.PI),
                // new WaitCommand(0.5),
                // VasiliWaypointTrajectory
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // controlVectors,
                // () -> new Rotation2d(Math.PI),
                // "output/GoBackToStation(x).wpilib.json")
                // new Rotate(m_robotDrive, 0)

                new SetCubeMode(arm, indicator),
                new ParallelDeadlineGroup(
                        new WaitCommand(3),
                        new ArmTrajectory(ArmPosition.HIGH, arm, false)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new Intake(manipulator)),
                new ParallelDeadlineGroup(
                        new WaitCommand(2),
                        new ArmTrajectory(ArmPosition.SAFE, arm, false)),

                new VasiliWaypointTrajectory(
                        drive,
                        kinematics,
                        () -> new Rotation2d(Math.PI),
                        gyro,
                        "output/BlueLeftExit.wpilib.json"),

                new AutoLevel(true, drive, gyro, chassisSpeedFactory));
    }
}
