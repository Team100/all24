package org.team100.commands;

import org.team100.Debug;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.planner.Drive;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;

import edu.wpi.first.math.geometry.Pose2d;
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
    private final IndexerSubsystem m_indexer;
    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final boolean m_debug;
    private final Tactics m_tactics;

    public DriveToNote(
            IndexerSubsystem indexer,
            DriveSubsystem drive,
            CameraSubsystem camera,
            Tactics tactics,
            boolean debug) {
        Arg.nonnull(indexer);
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        m_indexer = indexer;
        m_drive = drive;
        m_camera = camera;
        m_debug = debug && Debug.enable();
        m_tactics = tactics;
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
            m_drive.drive(m_tactics.finish(new FieldRelativeVelocity(0, 0, 0)));
            return;
        }

        // found a note

        FieldRelativeVelocity desired = Drive.goToGoalAligned(
                m_tactics,
                m_indexer,
                pose,
                closestSighting.position(),
                m_debug);

        m_drive.drive(desired);
    }
}
