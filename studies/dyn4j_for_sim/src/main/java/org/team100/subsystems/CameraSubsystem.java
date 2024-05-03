package org.team100.subsystems;

import java.util.NavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.dyn4j.geometry.Vector2;
import org.team100.sim.Body100;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This is a subsystem in order to get periodic(). */
public class CameraSubsystem extends SubsystemBase {
    public static record RobotSighting(boolean friend, Translation2d position) {
    }

    public static record NoteSighting(Translation2d position) {
    }

    /**
     * how old can sightings be and still be trusted?
     */
    private static final double kLookbackSec = 0.1;

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

    /**
     * In reality, each robot would get camera updates asynchronously (and
     * potentially out-of-order) but to keep it simple (and because dyn4j is not
     * thread safe), it's here.
     */
    private void lookForRobots() {
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
                Vector2 notePosition = body.getWorldCenter();
                double distance = position.distance(notePosition);
                // can't see that far
                if (distance > 5)
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

    public NavigableMap<Double, RobotSighting> recentSightings() {
        return sightings.descendingMap();
    }
}
