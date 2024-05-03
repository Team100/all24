package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a good spot for shooting.
 * 
 * This command is very approximate about location.
 * 
 * It should be followed by RotateToShoot.
 * 
 * TODO: extract a "drive to X" command
 */
public class DriveToSpeaker extends Command {
    private static final double kTranslationTolerance = 2;
    private static final double kVelocityTolerance = 5;
    private static final double kAngularTolerance = 2;
    // TODO: get these from kinodynamics
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s
    private static final double kAngularP = 10;
    private static final double kCartesianP = 5;
    private final DriveSubsystem m_drive;
    private final Pose2d m_goal;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToSpeaker(DriveSubsystem drive, CameraSubsystem camera, Pose2d goal, boolean debug) {
        m_drive = drive;
        m_goal = goal;
        m_debug = debug;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    /** TODO: replace with a more general driving plan */
    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToSpeaker");
        FieldRelativeVelocity desired = goToGoal();
        if (m_debug)
            ForceViz.put("desired", m_drive.getPose(), desired);
        if (m_debug)
            System.out.printf(" desired v %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired, true, true, m_debug);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(kMaxVelocity, kMaxOmega);
        if (m_debug)
            System.out.printf(" final v %s\n", v);
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

        return translationError < kTranslationTolerance
                && Math.abs(rotationError) < kAngularTolerance
                && velocity < kVelocityTolerance;
    }

    /** Proportional feedback with a limiter. */
    private FieldRelativeVelocity goToGoal() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB).clamp(kMaxVelocity, kMaxOmega);
    }

}
