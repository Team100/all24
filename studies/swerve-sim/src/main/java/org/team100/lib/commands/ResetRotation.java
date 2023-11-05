package org.team100.lib.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

/** Reset the rotation of the robot pose to the specified rotation. */
public class ResetRotation extends Command {
    private final Drivetrain robotDrive;
    private final Rotation2d robotRotation;
    private boolean done = false;

    public ResetRotation(Drivetrain drivetrain, Rotation2d rotation) {
        robotDrive = drivetrain;
        robotRotation = rotation;
    }

    @Override
    public void initialize() {
        System.out.println("reset rotation init");
        robotDrive.resetPose(
            new Pose2d(robotDrive.getPose().getTranslation(), robotRotation));
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("reset rotation end");

    }
}