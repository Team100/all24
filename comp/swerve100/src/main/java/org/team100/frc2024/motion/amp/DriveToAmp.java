package org.team100.frc2024.motion.amp;

import java.util.Optional;

import org.team100.frc2024.motion.FeedToAmp;
import org.team100.frc2024.motion.FeederSubsystem;
import org.team100.frc2024.motion.intake.Intake;
import org.team100.frc2024.motion.shooter.DrumShooter;
import org.team100.lib.commands.drivetrain.DriveWithProfile2;
import org.team100.lib.controller.HolonomicDriveController100;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.telemetry.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveToAmp extends SequentialCommandGroup {
    public DriveToAmp(
            Logger parent,
            SwerveDriveSubsystem drive,
            SwerveKinodynamics limits,
            AmpPivot amp,
            AmpFeeder ampFeeder,
            Intake intake,
            DrumShooter shooter,
            FeederSubsystem feeder) {

        Optional<Alliance> optAlliance = DriverStation.getAlliance();
        if (optAlliance.isEmpty()) {
            return;
        }
        Alliance alliance = optAlliance.get();
        if (alliance == Alliance.Blue) {
            addCommands(
                    new DriveWithProfile2(parent, () -> new Pose2d(1.834296, 6.474794, new Rotation2d(Math.PI / 2)),
                            drive,
                            new HolonomicDriveController100(parent), limits),
                    new ParallelCommandGroup(new AmpSet(parent, amp, 1.8),
                            new SequentialCommandGroup(
                                    new DriveWithProfile2(parent,
                                            () -> new Pose2d(1.834296, 6.799454, new Rotation2d(Math.PI / 2)), drive,
                                            new HolonomicDriveController100(parent), limits),
                                    new ParallelCommandGroup(ampFeeder.run(ampFeeder::outtake), new WaitCommand(1)))));
        } else {
            addCommands(
                    new ParallelDeadlineGroup(
                            new DriveWithProfile2(parent,
                                    () -> new Pose2d(1.834296, 8.221 - 6.474794, new Rotation2d(Math.PI / 2)),
                                    drive,
                                    new HolonomicDriveController100(parent), limits),
                            new FeedToAmp(intake, shooter, ampFeeder, feeder)),
                    new ParallelCommandGroup(new AmpSet(parent, amp, 1.8),
                            new SequentialCommandGroup(
                                    new DriveWithProfile2(parent,
                                            () -> new Pose2d(1.834296, 8.221 - 6.799454, new Rotation2d(Math.PI / 2)),
                                            drive,
                                            new HolonomicDriveController100(parent), limits),
                                    new ParallelCommandGroup(ampFeeder.run(ampFeeder::outtake), new WaitCommand(1)))));
        }

    }
}
