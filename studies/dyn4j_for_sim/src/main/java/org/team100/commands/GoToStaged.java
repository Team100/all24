package org.team100.commands;

import java.util.Optional;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.control.Pilot;
import org.team100.field.StagedNote;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive to a staged note location */
public class GoToStaged extends Command {
    private static final double kAngularP = 10;
    private static final double kCartesianP = 5;

    private final Pilot m_pilot;
    private final DriveSubsystem m_drive;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public GoToStaged(
            Pilot pilot,
            DriveSubsystem drive,
            CameraSubsystem camera,
            boolean debug) {
        Arg.nonnull(pilot);
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_pilot = pilot;
        m_drive = drive;
        m_debug = debug;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drive.getPose();
        int goalNoteId = m_pilot.goalNote();
        Optional<StagedNote> n = StagedNote.get(goalNoteId);
        if (n.isEmpty())
            return;
        Pose2d notePose = new Pose2d(
                n.get().getLocation().getX(),
                n.get().getLocation().getY(),
                pose.getRotation());
        FieldRelativeVelocity desired = goToGoal(pose, notePose);
        FieldRelativeVelocity v = m_tactics.apply(desired, true, true, true, m_debug && Debug.print());
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        m_drive.drive(v);
    }

    private FieldRelativeVelocity goToGoal(Pose2d pose, Pose2d m_goal) {
        FieldRelativeDelta transform = FieldRelativeDelta.delta(pose, m_goal);
        Vector2 positionError = new Vector2(transform.getX(), transform.getY());
        double rotationError = MathUtil.angleModulus(transform.getRotation().getRadians());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        double angularU_FB = rotationError * kAngularP;
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, angularU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

}
