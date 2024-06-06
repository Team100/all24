package org.team100.commands;

import java.util.Optional;

import org.team100.Debug;
import org.team100.control.Pilot;
import org.team100.field.StagedNote;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Drive to a staged note location */
public class GoToStaged extends Command {
    private static final double kAngularP = 5;
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
        m_debug = debug && Debug.enable();
        m_tactics = new Tactics(drive, camera, debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("GoToStaged");
        Pose2d pose = m_drive.getPose();
        if (m_debug)
            System.out.printf(" pose (%5.2f,%5.2f)", pose.getX(), pose.getY());
        int goalNoteId = m_pilot.goalNote();
        if (goalNoteId == 0)
            return;
        Optional<StagedNote> n = StagedNote.get(goalNoteId);
        if (n.isEmpty())
            return;

        // TODO: dedupe with drivetonote
        Translation2d targetFieldRelative = n.get().getLocation();
        if (m_debug)
            System.out.printf(" goal (%5.2f,%5.2f)", targetFieldRelative.getX(), targetFieldRelative.getY());
        Translation2d robotToTargetFieldRelative = targetFieldRelative.minus(pose.getTranslation());
        Rotation2d robotToTargetAngleFieldRelative = robotToTargetFieldRelative.getAngle();

        // intake is on the back
        Rotation2d intakeAngleFieldRelative = GeometryUtil.flip(pose.getRotation());
        double angleError = MathUtil.angleModulus(
                robotToTargetAngleFieldRelative.minus(intakeAngleFieldRelative).getRadians());

        boolean aligned = IndexerSubsystem.aligned(angleError);

        Translation2d cartesianU_FB = getCartesianU_FB(
                robotToTargetFieldRelative,
                aligned);

        double angleU_FB = angleError * kAngularP;

        FieldRelativeVelocity desired = new FieldRelativeVelocity(cartesianU_FB.getX(), cartesianU_FB.getY(), angleU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);

        // need to turn? avoid the edges.
        finish(desired, !aligned);
    }

    // TODO: dedupe with drivetonote
    // maybe this belongs in the drivetrain class
    private void finish(FieldRelativeVelocity desired, boolean avoidEdges) {
        if (m_debug)
            ForceViz.put("desired", m_drive.getPose(), desired);
        if (m_debug)
            System.out.printf(" desire %s", desired);

        FieldRelativeVelocity v = m_tactics.apply(desired, true, avoidEdges, true);

        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);

        if (m_debug)
            System.out.printf(" final %s\n", v);
        m_drive.drive(v);
    }

    // TODO: dedupe with drivetonote

    private Translation2d getCartesianU_FB(Translation2d robotToTargetFieldRelative, boolean aligned) {
        double distance = robotToTargetFieldRelative.getNorm();
        if (distance < IndexerSubsystem.kMinPickDistanceM || !aligned) {
            // target distance is lower than the tangent point: we ran the note
            // over without picking it, so back up.
            // also back up if not aligned.
            if (m_debug)
                System.out.print(" unaligned");
            double targetDistance = distance - 1;
            Translation2d targetTranslation = robotToTargetFieldRelative.times(targetDistance);
            return targetTranslation.times(kCartesianP);
        }
        if (m_debug)
            System.out.print(" aligned");

        // aligned, drive over the note
        return robotToTargetFieldRelative.times(kCartesianP);
    }

}
