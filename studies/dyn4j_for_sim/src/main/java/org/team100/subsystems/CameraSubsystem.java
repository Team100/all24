package org.team100.subsystems;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.dyn4j.geometry.Vector2;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Makes lists of robots and nearby notes it can see. */
public class CameraSubsystem extends SubsystemBase {
    /** Ignore note sightings further away than this. */
    private static final double kMaxNoteDistance = 5;
    /** Ignore robot sightings further away than this. */
    private static final double kMaxRobotDistance = 5;

    public static record RobotSighting(boolean friend, Translation2d position) {
    }

    public static record NoteSighting(Translation2d position) {
    }

    /**
     * how old can sightings be and still be trusted?
     */
    private static final double kLookbackSec = 0.2;

    /**
     * Some recent sightings from the camera system, used for robot avoidance and
     * defense.
     * 
     * We can't trust that the camera knows the identity of each sighting, just the
     * position (within some tolerance) and the time (quite precisely). We can also
     * detect friend-or-foe since the bumper color tells us. Key is time in sec.
     * Note: some time jitter should be used here since one camera frame may include
     * multiple sightings.
     */
    private NavigableMap<Double, RobotSighting> sightings = new ConcurrentSkipListMap<>();

    /** Recent note sightings. */
    private NavigableMap<Double, NoteSighting> noteSightings = new ConcurrentSkipListMap<>();

    private final RobotBody m_robotBody;

    public CameraSubsystem(RobotBody robotBody) {
        m_robotBody = robotBody;
    }

    @Override
    public void periodic() {
        lookForRobots();
        lookForNotes();
        trimSightings();
    }

   
    public NoteSighting findClosestNote(Pose2d pose) {
        // This map of notes is ordered by sighting age, not distance, so we need to
        // look at all of them.
        NavigableMap<Double, NoteSighting> notes = recentNoteSightings();
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
     * Key is timestamp in seconds.
     */
    public NavigableMap<Double, RobotSighting> recentSightings() {
        return sightings.descendingMap();
    }

    /**
     * Key is timestamp in seconds.
     */
    public NavigableMap<Double, NoteSighting> recentNoteSightings() {
        return noteSightings.descendingMap();
    }

    //////////////////////////////////////////////////////////////////

    /**
     * In reality, each robot would get camera updates asynchronously (and
     * potentially out-of-order) but to keep it simple (and because dyn4j is not
     * thread safe), it's here.
     */
    private void lookForRobots() {
        Vector2 position = m_robotBody.getWorldCenter();

        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body == m_robotBody) {
                // skip ourselves
                continue;
            }
            if (!(body instanceof RobotBody)) {
                // look only at robots
                continue;
            }
            RobotBody robotBody = (RobotBody) body;
            Vector2 targetPosition = robotBody.getWorldCenter();
            double distance = position.distance(targetPosition);
            // can't see that far
            if (distance > kMaxRobotDistance)
                continue;
            boolean friend = robotBody.friend(m_robotBody);
            addSighting(friend, targetPosition);
        }
    }

    /**
     * Camera looking for notes, sticking them in a buffer of note sightings. As
     * above this would actually be asynchronous.
     */
    private void lookForNotes() {
        Vector2 position = m_robotBody.getWorldCenter();
        // look for nearby notes, brute force
        for (Body100 body : m_robotBody.getWorld().getBodies()) {
            if (body instanceof Note) {
                if (!((Note) body).isVisible()) {
                    // ignore notes carried by other robots, or flying through the air.
                    continue;
                }
                Vector2 notePosition = body.getWorldCenter();
                double distance = position.distance(notePosition);
                // can't see that far
                if (distance > kMaxNoteDistance)
                    continue;
                double now = Timer.getFPGATimestamp();
                NoteSighting sighting = new NoteSighting(
                        new Translation2d(notePosition.x, notePosition.y));
                noteSightings.put(now, sighting);
            }
        }
    }

    /** Add a sighting with the current timestamp. */
    private void addSighting(boolean friend, Vector2 fieldRelativePosition) {
        double now = Timer.getFPGATimestamp();
        RobotSighting sighting = new RobotSighting(
                friend,
                new Translation2d(fieldRelativePosition.x, fieldRelativePosition.y));
        sightings.put(now, sighting);
    }

    /** Don't remember stale sightings. */
    private void trimSightings() {
        double now = Timer.getFPGATimestamp();
        sightings.keySet().removeAll(sightings.headMap(now - kLookbackSec).keySet());
        noteSightings.keySet().removeAll(noteSightings.headMap(now - kLookbackSec).keySet());
    }

}
