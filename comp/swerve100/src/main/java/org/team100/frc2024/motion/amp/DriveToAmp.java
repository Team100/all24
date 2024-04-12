package org.team100.frc2024.motion.amp;

import org.team100.lib.commands.drivetrain.DriveWithProfile2;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveToAmp extends SequentialCommandGroup {
    public DriveToAmp(
            SwerveDriveSubsystem drive,
            SwerveKinodynamics limits,
            Alliance alliance) {

        if (alliance == Alliance.Blue) {
            addCommands(new DriveWithProfile2(() -> new Pose2d(1.834296, 7.474794, new Rotation2d(Math.PI / 2)), drive,
                    new HolonomicDriveController100(), limits),
                    new DriveWithProfile2(() -> new Pose2d(1.834296, 7.799454, new Rotation2d(Math.PI / 2)), drive,
                            new HolonomicDriveController100(), limits));
        } else {
            addCommands(
                    new DriveWithProfile2(() -> new Pose2d(1.834296, 8.221 - 7.474794, new Rotation2d(Math.PI / 2)),
                            drive,
                            new HolonomicDriveController100(), limits),
                    new DriveWithProfile2(() -> new Pose2d(1.834296, 8.221 - 7.799454, new Rotation2d(Math.PI / 2)),
                            drive,
                            new HolonomicDriveController100(), limits)

            );
        }

    }
}
