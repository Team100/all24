package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command should be followed by DriveToNote, so that the goal of this
 * command can be very approximate.
 */
public class DriveToSource extends Command {
    /** The intake works at high angles. */
    private static final double kAngularTolerance = 0.75;
    /** Velocity doesn't matter at all. */
    private static final double kVelocityTolerance = 5;
    /** Get close enough for the camera to see. */
    private static final double kCartesianTolerance = 2.5;
    private static final double kAngularP = 10;
    private static final double kCartesianP = 5;
    private final DriveSubsystem m_drive;
    private final Pose2d m_goal;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToSource(
            DriveSubsystem drive,
            CameraSubsystem camera,
            boolean debug) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_drive = drive;
        m_goal = m_drive.sourcePosition();
        m_debug = debug;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug && Debug.print())
            System.out.print("DriveToSource");
        Pose2d pose = m_drive.getPose();
        if (m_debug && Debug.print())
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        FieldRelativeVelocity desired = goToGoal(pose);
        if (m_debug)
            ForceViz.put("desired", pose, desired);
        if (m_debug && Debug.print())
            System.out.printf(" desired %s", desired);
        FieldRelativeVelocity v = m_tactics.apply(desired, true, false, true, m_debug && Debug.print());
        if (m_debug && Debug.print())
            System.out.printf(" tactics %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug && Debug.print())
            System.out.printf(" final %s\n", v);
        m_drive.drive(v);
    }

    /** TODO: i think this never matters, the pilot cancels the command */
    @Override
    public boolean isFinished() {
        Pose2d pose = m_drive.getPose();
        FieldRelativeDelta t = FieldRelativeDelta.delta(pose, m_goal);
        double translationError = t.getTranslation().getNorm();
        double rotationError = t.getRotation().getRadians();
        double velocity = m_drive.getVelocity().norm();
        return translationError < kCartesianTolerance
                && Math.abs(rotationError) < kAngularTolerance
                && velocity < kVelocityTolerance;
    }

    private FieldRelativeVelocity goToGoal(Pose2d pose) {
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

}
