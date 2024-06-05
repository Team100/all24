package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.control.Pilot;
import org.team100.kinodynamics.Kinodynamics;
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
 * This command uses loose tolerance since the exact position doesn't matter;
 * the actual range is used for the shooter angle.
 * 
 * RotateToShoot should be used to rotate to the correct angle given the actual
 * location, and come to a complete stop.
 * 
 * TODO: extract a "drive to X" command
 */
public class DriveToSpeaker extends Command {
    private static final double kTranslationTolerance = 0.75;
    private static final double kVelocityTolerance = 1;
    private static final double kAngularTolerance = 0.25;
    private static final double kAngularP = 10;
    private static final double kCartesianP = 5;
    private final Pilot m_pilot;
    private final DriveSubsystem m_drive;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToSpeaker(Pilot pilot,
            DriveSubsystem drive,
            CameraSubsystem camera,
            boolean debug) {
        m_pilot = pilot;
        m_drive = drive;
        m_debug = debug && Debug.enable();
        m_tactics = new Tactics(drive, camera, debug);
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
        FieldRelativeVelocity v = m_tactics.apply(desired, true, true, true, m_debug);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
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
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_pilot.shootingLocation());
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
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_pilot.shootingLocation());
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

}
