package org.team100.commands;

import java.util.NavigableMap;
import java.util.Map.Entry;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to the nearest note.
 * 
 * This never finishes; run it with an Intake command as the deadline.
 */
public class DriveToNote extends Command {
    // TODO: get these from kinodynamics
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s
    private static final int kCartesianP = 50;

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
            System.out.printf("DriveToNote\n");
        // some notes might be near the edge, so turn off edge repulsion.
        FieldRelativeVelocity v = m_tactics.apply(false, true, false);
        if (m_debug)
            System.out.printf("tactics v %s\n", v);
        v = v.plus(goToGoal());
        v = v.clamp(kMaxVelocity, kMaxOmega);
        if (m_debug)
            System.out.printf("final v %s\n", v);
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
            System.out.printf("DriveToNote pose %s target %s\n",
                    m_drive.getPose().getTranslation(), closest);

        Vector2 positionError = new Vector2(closest.getX(), closest.getY());
        final int maxError = 1;
        positionError = new Vector2(
                MathUtil.clamp(positionError.x, -maxError, maxError),
                MathUtil.clamp(positionError.y, -maxError, maxError));
        Vector2 cartesianU_FB = positionError.product(kCartesianP);
        return new FieldRelativeVelocity(cartesianU_FB.x, cartesianU_FB.y, 0);
    }

}
