package org.team100.commands;

import java.util.NavigableMap;
import java.util.Map.Entry;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the nearest note.
 * 
 * This never finishes; run it with an Intake command as the deadline.
 */
public class DriveToNote extends Command {
    /** Ignore notes further than this. Note the camera has a limit too. */
    private static final int kMaxNoteDistance = 5;
    // TODO: get these from kinodynamics
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s
    private static final int kCartesianP = 5;

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
        if (m_debug)
            System.out.print("DriveToNote");
        FieldRelativeVelocity desired = goToGoal();
        if (m_debug)
            ForceViz.put("desired", m_drive.getPose(), desired);
        if (m_debug)
            System.out.printf(" desired v %s", desired);
        // some notes might be near the edge, so turn off edge repulsion.
        FieldRelativeVelocity v = m_tactics.apply(desired, false, true, m_debug);
        if (m_debug)
            System.out.printf(" tactics v %s", v);
        v = v.plus(desired);
        v = v.clamp(kMaxVelocity, kMaxOmega);
        if (m_debug)
            System.out.printf(" final v %s\n", v);
        m_drive.drive(v);
    }

    /**
     * Go to the closest note, irrespective of the age of the sighting (since notes
     * don't move and sightings are all pretty new)
     */
    private FieldRelativeVelocity goToGoal() {
        // TODO: use a future estimated pose to account for current velocity
        Pose2d pose = m_drive.getPose();
        // this is arranged by sighting age, not distance.
        NavigableMap<Double, NoteSighting> notes = m_camera.recentNoteSightings();
        double minDistance = Double.MAX_VALUE;
        Translation2d closest = null;
        for (Entry<Double, NoteSighting> entry : notes.entrySet()) {
            NoteSighting sight = entry.getValue();
            Translation2d translation = sight.position().minus(pose.getTranslation());
            double distance = translation.getNorm();
            if (distance > kMaxNoteDistance)
                continue;
            if (distance < minDistance) {
                minDistance = distance;
                closest = translation;
            }
        }
        if (closest == null) {
            // no nearby note
            return new FieldRelativeVelocity(0, 0, 0);
        }
        // found a target
        // TODO: something about rotation

        if (m_debug)
            System.out.printf(" pose (%5.2f, %5.2f) target (%5.2f, %5.2f)",
                    pose.getX(), pose.getY(), closest.getX(), closest.getY());

        Vector2 positionError = new Vector2(closest.getX(), closest.getY());
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, 0).clamp(kMaxVelocity, kMaxOmega);
    }

}
