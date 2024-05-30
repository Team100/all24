package org.team100.commands;

import java.util.NavigableMap;
import java.util.Map.Entry;

import org.team100.Debug;
import org.team100.kinodynamics.Kinodynamics;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the nearest note, turning so that the intake side arrives first.
 * 
 * If the robot is against the wall, the turn has trouble. TODO: fix that
 * 
 * If the robot is between two notes, it switches between them and makes no
 * progress. TODO: fix that.
 * 
 * This never finishes; run it with an Intake command as the deadline.
 */
public class DriveToNote extends Command {
    /** Ignore notes further than this. Note the camera has a limit too. */
    private static final double kMaxNoteDistance = 5;
    private static final double kCartesianP = 5;
    private static final double kRotationP = 5;
    /** Intake admittance half-angle. */
    private static final double kAdmittanceRad = 0.1;
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
        m_drive = drive;
        m_camera = camera;
        m_debug = debug;
        m_tactics = new Tactics(drive, camera);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (m_debug && Debug.print())
            System.out.print("DriveToNote");
        FieldRelativeVelocity desired = goToGoal();
        if (m_debug)
            ForceViz.put("desired", m_drive.getPose(), desired);
        if (m_debug && Debug.print())
            System.out.printf(" desire %s", desired);
        // some notes might be near the edge, so turn off edge repulsion.
        FieldRelativeVelocity v = m_tactics.apply(desired, true, false, true, m_debug && Debug.print());
        if (m_debug && Debug.print())
            System.out.printf(" tactic %s", v);
        v = v.plus(desired);
        v = v.clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
        if (m_debug && Debug.print())
            System.out.printf(" final %s\n", v);
        m_drive.drive(v);
    }

    /**
     * TODO: remember and prefer the previous fixation, unless some new sighting is
     * much better.
     */
    NoteSighting findClosestNote(Pose2d pose) {
        // This map of notes is ordered by sighting age, not distance, so we need to
        // look at all of them.
        NavigableMap<Double, NoteSighting> notes = m_camera.recentNoteSightings();
        double minDistance = Double.MAX_VALUE;
        NoteSighting closestSighting = null;
        for (Entry<Double, NoteSighting> entry : notes.entrySet()) {
            NoteSighting sight = entry.getValue();
            double distance = sight.position().getDistance(pose.getTranslation());
            if (distance > kMaxNoteDistance) {
                // ignore far-away notes
                continue;
            }
            if (distance < minDistance) {
                minDistance = distance;
                closestSighting = sight;
            }
        }
        return closestSighting;
    }

    /**
     * Go to the closest note, irrespective of the age of the sighting (since notes
     * don't move and sightings are all pretty new)
     */
    private FieldRelativeVelocity goToGoal() {
        // TODO: use a future estimated pose to account for current velocity
        Pose2d pose = m_drive.getPose();

        NoteSighting closestSighting = findClosestNote(pose);
        if (closestSighting == null) {
            // no nearby note, no need to move
            return new FieldRelativeVelocity(0, 0, 0);
        }

        // found a note

        Translation2d targetFieldRelative = closestSighting.position();
        if (m_debug && Debug.print())
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), targetFieldRelative.getX(), targetFieldRelative.getY());

        Translation2d robotToTargetFieldRelative = targetFieldRelative.minus(pose.getTranslation());
        Rotation2d robotToTargetAngleFieldRelative = robotToTargetFieldRelative.getAngle();
        // intake is on the back
        Rotation2d intakeAngleFieldRelative = GeometryUtil.flip(pose.getRotation());
        double angleError = MathUtil.angleModulus(
                robotToTargetAngleFieldRelative.minus(intakeAngleFieldRelative).getRadians());

        Translation2d positionError = robotToTargetFieldRelative;

        Translation2d cartesianU_FB = getCartesianU_FB(robotToTargetFieldRelative, angleError, positionError);

        double angleU_FB = angleError * kRotationP;

        // we also want to turn the intake towards the note
        return new FieldRelativeVelocity(cartesianU_FB.getX(), cartesianU_FB.getY(), angleU_FB)
                .clamp(Kinodynamics.kMaxVelocity, Kinodynamics.kMaxOmega);
    }

    /** Go to the note if aligned, If not, go 1m away. */
    private Translation2d getCartesianU_FB(
            Translation2d robotToTargetFieldRelative,
            double angleError,
            Translation2d positionError) {
        if (Math.abs(angleError) < kAdmittanceRad) {
            // aligned, go to the center
            return positionError.times(kCartesianP);
        } else {
            // not aligned, go to 1m away
            double distance = robotToTargetFieldRelative.getNorm();
            double targetDistance = distance - kPickRadius;
            Translation2d targetTranslation = robotToTargetFieldRelative.times(targetDistance);
            return targetTranslation.times(kCartesianP);
        }
    }

}
