package org.team100.commands;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.util.Arg;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the nearest note, turning so that the intake side arrives first.
 * 
 * If the robot is between two notes, it switches between them and makes no
 * progress. TODO: fix that.
 * 
 * This never finishes; run it with an Intake command as the deadline.
 */
public class DriveToNote extends Command {
    private static final double kCartesianP = 5;
    private static final double kRotationP = 5;
    /** Go this far from the note until rotated correctly. */
    private static final double kPickRadius = 1;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToNote(
            DriveSubsystem drive,
            CameraSubsystem camera,
            boolean debug) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_drive = drive;
        m_camera = camera;
        m_debug = debug && Debug.enable();
        m_tactics = new Tactics(drive, camera, debug);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug)
            System.out.print("DriveToNote");

        // where are we with respect to the goal?

        // TODO: use a future estimated pose to account for current velocity

        Pose2d pose = m_drive.getPose();

        goToGoal(pose);
    }

  
    /**
     * Go to the closest note, irrespective of the age of the sighting (since notes
     * don't move and sightings are all pretty new)
     */
    private void goToGoal(Pose2d pose) {

        // TODO: remember and prefer the previous fixation, unless some new sighting is
        // much better.

        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            // no nearby note, no need to move
            m_drive.drive(m_tactics.finish(new FieldRelativeVelocity(0, 0, 0), true, false, true));
            return;
        }

        // found a note

        Translation2d targetFieldRelative = closestSighting.position();
        if (m_debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), targetFieldRelative.getX(), targetFieldRelative.getY());

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

        double angleU_FB = angleError * kRotationP;

        // we also want to turn the intake towards the note
        FieldRelativeVelocity desired = new FieldRelativeVelocity(cartesianU_FB.getX(), cartesianU_FB.getY(), angleU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);

        // need to turn? avoid the edges.
        m_drive.drive(m_tactics.finish(desired, true, !aligned, true));
    }

    /** Go to the note if aligned. If not, or if we missed, go 1m away. */
    private Translation2d getCartesianU_FB(Translation2d robotToTargetFieldRelative, boolean aligned) {
        double distance = robotToTargetFieldRelative.getNorm();
        if (distance < IndexerSubsystem.kMinPickDistanceM || !aligned) {
            // target distance is lower than the tangent point: we ran the note
            // over without picking it, so back up.
            // also back up if not aligned.
            double targetDistance = distance - kPickRadius;
            Translation2d targetTranslation = robotToTargetFieldRelative.times(targetDistance);
            return targetTranslation.times(kCartesianP);
        }

        // aligned, drive over the note
        return robotToTargetFieldRelative.times(kCartesianP);
    }

}
