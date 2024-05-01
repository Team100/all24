package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a good spot for shooting.
 * 
 * TODO: extract a "drive to X" command
 */
public class DriveToSpeaker extends Command {
    private static final int kAngularP = 10;
    private static final int kCartesianP = 50;
    private final DriveSubsystem m_drive;
    private final Pose2d m_goal;
    private final Tactics m_tactics;

    public DriveToSpeaker(DriveSubsystem drive, CameraSubsystem camera, Pose2d goal) {
        m_drive = drive;
        m_goal = goal;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    /** TODO: replace with a more general driving plan */
    @Override
    public void execute() {
        FieldRelativeVelocity v = m_tactics.apply(true, true, false);
        v = v.plus(goToGoal());
        m_drive.drive(v);
    }

    /**
     * speaker position tolerance is loose but angle is not
     */
    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_goal);
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        double velocity = m_drive.getVelocity().norm();

        return translationError < 0.5
                && Math.abs(rotationError) < 0.05
                && velocity < 0.05;
    }

    /** Proportional feedback with a limiter. */
    private FieldRelativeVelocity goToGoal() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        final int maxError = 1;
        positionError = new Vector2(
                MathUtil.clamp(positionError.x, -maxError, maxError),
                MathUtil.clamp(positionError.y, -maxError, maxError));
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB);
    }

}
