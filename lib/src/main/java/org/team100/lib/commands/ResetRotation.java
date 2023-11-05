package org.team100.lib.commands;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Reset the rotation of the robot pose to the specified rotation. */
public class ResetRotation extends Command {
    private final SwerveDriveSubsystem robotDrive;
    private final Rotation2d robotRotation;
    private boolean done = false;

    public ResetRotation(SwerveDriveSubsystem drivetrain, Rotation2d rotation) {
        robotDrive = drivetrain;
        robotRotation = rotation;
    }

    @Override
    public void initialize() {
        robotDrive.resetPose(new Pose2d(robotDrive.getPose().getTranslation(), robotRotation));
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
