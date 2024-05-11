package org.team100.commands;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stop and turn to the speaker.
 * 
 * TODO: make a shoot-on-the-move command
 */
public class RotateToShoot extends Command {
    private static final double kAngleTolerance = 0.05;
    private static final double kVelocityTolerance = 0.05;
    private static final double kAngularP = 10;
    private static final double kOmegaP = 1;
    private static final double kVelocityP = 1;
    private final Translation2d m_speakerPosition;
    private final DriveSubsystem m_drive;
    private final boolean m_debug;

    public RotateToShoot(Translation2d speakerPosition, DriveSubsystem drive, boolean debug) {
        m_speakerPosition = speakerPosition;
        m_drive = drive;
        m_debug = debug;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug && Debug.print())
            System.out.print("RotateToShoot");
        FieldRelativeVelocity goalVelocity = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity velocityError = goalVelocity.minus(m_drive.getVelocity());
        FieldRelativeVelocity velocityFeedback = velocityError.times(kVelocityP, kOmegaP);
        if (m_debug && Debug.print())
            System.out.printf(" v_FB %s", velocityFeedback);

        Pose2d pose = m_drive.getPose();
        double goalAngle = m_speakerPosition.minus(pose.getTranslation()).getAngle().getRadians();
        double angularError = MathUtil.angleModulus(goalAngle - pose.getRotation().getRadians());
        FieldRelativeVelocity angularFeedback = new FieldRelativeVelocity(0, 0, angularError * kAngularP);
        if (m_debug && Debug.print())
            System.out.printf(" a_FB %s", angularFeedback);

        FieldRelativeVelocity v = velocityFeedback.plus(angularFeedback)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug && Debug.print())
            System.out.printf(" final v %s\n", v);
        m_drive.drive(v);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        double angle = m_speakerPosition.minus(pose.getTranslation()).getAngle().getRadians();
        double error = MathUtil.angleModulus(angle - pose.getRotation().getRadians());
        double velocity = m_drive.getVelocity().norm();
        return error < kAngleTolerance && velocity < kVelocityTolerance;
    }
}
