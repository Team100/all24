package org.team100.lib.commands.drivetrain;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Set the rotation of the robot pose to the specified rotation.
 * 
 * This is good for resetting the "zero," in cases where the robot isn't
 * actually facing zero (e.g. 180)
 */
public class SetRotation extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final Rotation2d m_rotation;

    public SetRotation(SwerveDriveSubsystem drivetrain, Rotation2d rotation) {
        m_drive = drivetrain;
        m_rotation = rotation;
    }

    @Override
    public void initialize() {
        m_drive.resetPose(new Pose2d(m_drive.getState().pose().getTranslation(), m_rotation));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
