package org.team100.control.auto;

import org.team100.control.Pilot;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;
import org.team100.util.Counter;

import edu.wpi.first.math.geometry.Pose2d;

/** Fetch a note, shoot it into the speaker, repeat. */
public class Auton implements Pilot {
    private static final double kMaxNoteDistance = 8.0;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    private final Pose2d m_shooting;
    private final Integer[] m_notes;
    private final Counter m_counter;

    private boolean m_enabled = false;

    public Auton(
            DriveSubsystem drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer,
            Pose2d shooting,
            Integer... notes) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        Arg.nonempty(notes);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_notes = notes;
        m_shooting = shooting;
        m_counter = new Counter(m_indexer::full);
    }

    // first go to the right place
    @Override
    public boolean driveToStaged() {
        return m_enabled && !noteNearby() && !m_indexer.full();
    }

    // once we see a note, go to it ...
    @Override
    public boolean driveToNote() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    // ... and intake it
    @Override
    public boolean intake() {
        return m_enabled && noteNearby() && !m_indexer.full();
    }

    // if we have one, go score it.
    // there needs to be room for multiple scorers
    @Override
    public boolean scoreSpeaker() {
        return m_enabled && m_indexer.full();
    }

    @Override
    public Pose2d shootingLocation() {
        return m_shooting;
    }

    @Override
    public int goalNote() {
        // there is no goal note if we already have one
        if (m_indexer.full())
            return 0;
        return m_notes[m_counter.getAsInt()];
    }

    @Override
    public void begin() {
        m_enabled = true;
    }

    @Override
    public void reset() {
        m_enabled = false;
        m_counter.reset();
    }

    ///////////////////////////////////////////////////////////////////

    private boolean noteNearby() {
        Pose2d pose = m_drive.getPose();
        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            return false;
        }
        return closestSighting.position().getDistance(pose.getTranslation()) <= kMaxNoteDistance;
    }

}
