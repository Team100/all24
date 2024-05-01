package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.Source;
import org.team100.sim.Body100;
import org.team100.sim.Foe;
import org.team100.sim.Note;
import org.team100.sim.RobotBody;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Dumps a note on the field once a second, if there are friends around.
 */
public class SourceDefault extends Command {
    private static final double kMaxDistance = 2;

    private final Source m_humanPlayer;
    private final SimWorld m_world;
    private final boolean m_isBlue;
    private final boolean m_debug;

    public SourceDefault(Source source, SimWorld world, boolean isBlue, boolean debug) {
        m_humanPlayer = source;
        m_world = world;
        m_isBlue = isBlue;
        m_debug = debug;
        addRequirements(source);
    }

    @Override
    public void execute() {
        // feed if there's a nearby friend, but no nearby note
        if (nearFriend() && !nearNote()) {
            m_humanPlayer.feed();
        }
    }

    /**
     * True if any friends are nearby.
     */
    private boolean nearFriend() {
        for (Body100 body : m_world.getBodies()) {
            if (!(body instanceof RobotBody)) {
                // look only at robots
                continue;
            }
            Vector2 robotPosition = body.getWorldCenter();
            Translation2d robotTranslation = new Translation2d(robotPosition.x, robotPosition.y);
            double distance = robotTranslation.getDistance(m_humanPlayer.getTarget());
            if (distance > kMaxDistance) {// ignore distant robots
                continue;
            }

            if (m_isBlue && body instanceof Foe) {
                // blue source does not feed red robots.
                continue;
            }
            if (!m_isBlue && !(body instanceof Foe)) {
                // red source does not feed blue robots
                if (m_debug)
                    System.out.printf("ignoring %s\n", body);
                continue;
            }

            return true;
        }
        return false;
    }

    /** True if there are notes nearby. */
    private boolean nearNote() {
        for (Body100 body : m_world.getBodies()) {
            if (!(body instanceof Note)) {
                // look only at notes
                continue;
            }
            Vector2 notePosition = body.getWorldCenter();
            Translation2d noteTranslation = new Translation2d(notePosition.x, notePosition.y);
            double distance = noteTranslation.getDistance(m_humanPlayer.getTarget());
            if (distance > kMaxDistance) {// ignore distant notes
                continue;
            }
            return true;
        }
        return false;

    }
}
