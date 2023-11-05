package org.team100.frc2023.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class ResetRotation extends Command {
    /** Creates a new ResetAngle. */
    Drivetrain robotDrive;
    Rotation2d robotRotation;
    boolean done = false;

    public ResetRotation(Drivetrain swerve2DriveSubsystem, Rotation2d rotation) {
        // Use addRequirements() here to declare subsystem dependencies.
        robotDrive = swerve2DriveSubsystem;
        robotRotation = rotation;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        robotDrive.resetPose(new Pose2d(robotDrive.getPose().getTranslation(), robotRotation));
        done = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
