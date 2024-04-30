package org.team100.commands;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stop and turn to the speaker. This should optimally happen as part of another
 * driving behavior.
 */
public class RotateToShoot extends Command {
    private final Translation2d m_speakerPosition;
    private final DriveSubsystem m_drive;

    public RotateToShoot(Translation2d speakerPosition, DriveSubsystem drive) {
        m_speakerPosition = speakerPosition;
        m_drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drive.getPose();
        double angle = m_speakerPosition.minus(pose.getTranslation()).getAngle().getRadians();
        double error = MathUtil.angleModulus(angle - pose.getRotation().getRadians());
        FieldRelativeVelocity measurement = m_drive.getVelocity();
        double omega = error * 10;
        m_drive.drive(new FieldRelativeVelocity(0, 0, omega));
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        double angle = m_speakerPosition.minus(pose.getTranslation()).getAngle().getRadians();
        double error = MathUtil.angleModulus(angle - pose.getRotation().getRadians());
        double velocity = m_drive.getVelocity().norm();
        return error < 0.05 && velocity < 0.05;
    }
}
