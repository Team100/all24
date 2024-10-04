package org.team100.commands;

import java.util.function.Supplier;

import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Debug;
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
    private static final double kAngularP = 10;
    private static final double kOmegaP = 1;
    private static final double kVelocityP = 1;
    private final Supplier<Translation2d> m_speakerPosition;
    private final DriveSubsystemInterface m_drive;
    private final Tolerance m_tolerance;
    private final boolean m_debug;

    public RotateToShoot(
            DriveSubsystem drive,
            Supplier<Translation2d> target,
            Tolerance tolerance,
            boolean debug) {
        m_speakerPosition = target;
        m_drive = drive;
        m_tolerance = tolerance;
        m_debug = debug && Debug.enable();
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("RotateToShoot");
        FieldRelativeVelocity goalVelocity = new FieldRelativeVelocity(0, 0, 0);
        FieldRelativeVelocity velocityError = goalVelocity.minus(m_drive.getVelocity());
        FieldRelativeVelocity velocityFeedback = velocityError.times(kVelocityP, kOmegaP);
        if (m_debug)
            System.out.printf(" v_FB %s", velocityFeedback);

        Pose2d pose = m_drive.getPose();
        double goalAngle = m_speakerPosition.get().minus(pose.getTranslation()).getAngle().getRadians();
        double angularError = MathUtil.angleModulus(goalAngle - pose.getRotation().getRadians());
        FieldRelativeVelocity angularFeedback = new FieldRelativeVelocity(0, 0, angularError * kAngularP);
        if (m_debug)
            System.out.printf(" a_FB %s", angularFeedback);

        FieldRelativeVelocity v = velocityFeedback.plus(angularFeedback)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug)
            System.out.printf(" final v %s\n", v);
        m_drive.drive(v);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        double angle = m_speakerPosition.get().minus(pose.getTranslation()).getAngle().getRadians();
        double error = MathUtil.angleModulus(angle - pose.getRotation().getRadians());
        double velocity = m_drive.getVelocity().norm();
        double omega = m_drive.getVelocity().theta();
        return error < m_tolerance.kAngularTolerance() && velocity < m_tolerance.kVelocityTolerance()
                && omega < m_tolerance.kVelocityTolerance();
    }
}
