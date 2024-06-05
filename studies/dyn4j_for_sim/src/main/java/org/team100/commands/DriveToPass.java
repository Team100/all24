package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
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
 * Drive to a passing spot.
 * 
 * TODO: this works considerably better if the shot is taken while moving
 */
public class DriveToPass extends Command {
    private static final int kAngularP = 10;
    private static final int kCartesianP = 5;
    private final DriveSubsystem m_drive;
    private final Pose2d m_goal;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToPass(DriveSubsystem drive, CameraSubsystem camera, boolean debug) {
        m_drive = drive;
        m_goal = m_drive.passingPosition();
        m_debug = debug && Debug.enable();
        m_tactics = new Tactics(drive, camera, debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToPass execute");
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

    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_goal);
        double velocity = m_drive.getVelocity().norm();
        return t.getTranslation().getNorm() < 0.5
                && Math.abs(t.getRotation().getRadians()) < 0.1
                && velocity < 0.1;
    }

    /** Proportional feedback with a limiter. */
    private FieldRelativeVelocity goToGoal() {
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), m_goal.getX(), m_goal.getY());

        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

}
