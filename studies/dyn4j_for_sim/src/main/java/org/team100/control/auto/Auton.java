package org.team100.control.auto;

import java.util.Arrays;
import java.util.NavigableMap;
import java.util.Optional;

import org.team100.control.Pilot;
import org.team100.field.StagedNote;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.NoteSighting;
import org.team100.subsystems.DriveSubsystem;
import org.team100.subsystems.IndexerSubsystem;
import org.team100.util.Arg;
import org.team100.util.Counter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Fetch a note, shoot it into the speaker, repeat.
 * 
 * TODO: make a special "shoot preload" command.
 * 
 * The new design here will have an array for "state" of notes that remain to be
 * picked; each note state is a belief informed by observations and decayed by
 * the passage of time. The state unit is probability of presence, a crisp value
 * for the estimate.
 * 
 * So the choice of note to pursue depends on this belief.
 */
public class Auton implements Pilot {
    private static final double kBeliefUpdateTolerance = 0.1;

    // tolerance for go-to-staged-note paths.
    private static final double kStageTolerance = 0.6;

    // ignore notes further than this.
    // the limit is very small to prevent distraction.
    // TODO: what if the target note is missing?
    private static final double kMaxNoteDistance = 0.5;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final IndexerSubsystem m_indexer;
    private final Pose2d m_shooting;
    private final boolean m_debug;
    private final Integer[] m_notes;
    private final double[] m_beliefs;
    private final Counter m_counter;

    private boolean m_enabled = false;

    public Auton(
            DriveSubsystem drive,
            CameraSubsystem camera,
            IndexerSubsystem indexer,
            Pose2d shooting,
            boolean debug,
            Integer... notes) {
        Arg.nonnull(drive);
        Arg.nonnull(camera);
        Arg.nonnull(indexer);
        Arg.nonempty(notes);
        m_drive = drive;
        m_camera = camera;
        m_indexer = indexer;
        m_notes = notes;
        m_beliefs = new double[notes.length];
        // the initial state is certainty that each position is be full.
        Arrays.fill(m_beliefs, 1);
        m_shooting = shooting;
        m_debug = debug;
        m_counter = new Counter(m_indexer::full);
    }

    // first go to the right place, ignoring nearby notes on the way.
    @Override
    public boolean driveToStaged() {
        return m_enabled && !nearGoal() && !m_indexer.full();
    }

    // once we see a note, go to it ...
    @Override
    public boolean driveToNote() {
        return m_enabled && noteNearby() && nearGoal() && !m_indexer.full();
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
        int idx = m_counter.getAsInt();
        if (idx > m_notes.length - 1)
            return 0;
        Integer noteId = m_notes[idx];
        if (m_debug)
            System.out.println("goal note id " + noteId);
        return noteId;
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

    @Override
    public void periodic() {
        m_counter.periodic();
        updateBeliefs();
        // beliefs decay to zero over time.
        for (int i = 0; i < m_beliefs.length; ++i) {
            m_beliefs[i] *= 0.01;
        }
    }

    ///////////////////////////////////////////////////////////////////

    /**
     * Use vision to find nearby notes.
     * 
     * This provides both positive evidence (notes sighted) and negative evidence
     * (notes missing), if we know what the vision radius is.
     */
    private void updateBeliefs() {
        Pose2d robotPose = m_drive.getPose();
        double visionRadiusM = CameraSubsystem.kMaxNoteDistance;
        // (timestamp, sighting)
        NavigableMap<Double, NoteSighting> notes = m_camera.recentNoteSightings();

        // join our agenda to the sightings.
        for (int i = 0; i < m_notes.length; ++i) {
            int noteId = m_notes[i];
            Optional<StagedNote> n = StagedNote.get(noteId);
            if (n.isEmpty())
                continue;
            Translation2d noteLocation = n.get().getLocation();
            if (noteLocation.getDistance(robotPose.getTranslation()) > visionRadiusM) {
                // too far to see, do not update belief
                continue;
            }
            for (NoteSighting sighting : notes.values()) {
                // can we see it?
                if (noteLocation.getDistance(sighting.position()) < kBeliefUpdateTolerance) {
                    // found it!
                    m_beliefs[i] = 1;
                }
            }
            // it's gone!
            m_beliefs[i] = 0;
        }

    }

    private boolean noteNearby() {
        Pose2d pose = m_drive.getPose();
        NoteSighting closestSighting = m_camera.findClosestNote(pose);
        if (closestSighting == null) {
            return false;
        }
        return closestSighting.position().getDistance(pose.getTranslation()) <= kMaxNoteDistance;
    }

    // this should actually latch somehow.
    private boolean nearGoal() {
        Pose2d pose = m_drive.getPose();
        int idx = m_counter.getAsInt();
        if (idx > m_notes.length - 1)
            return false;
        Integer noteId = m_notes[idx];
        Optional<StagedNote> n = StagedNote.get(noteId);
        if (n.isEmpty())
            return false;
        Translation2d goal = n.get().getLocation();
        double distance = goal.getDistance(pose.getTranslation());
        return distance < kStageTolerance;
    }

}
