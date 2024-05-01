package org.team100.commands;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.Source;
import org.team100.sim.Body100;
import org.team100.sim.Foe;
import org.team100.sim.RobotBody;
import org.team100.sim.SimWorld;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Dumps a note on the field once a second, if there are friends around.
 */
public class SourceDefault extends Command {
    /** How often to add a note. */
    private static final double kPeriodS = 1;
    private static final double kMaxDistance = 2;

    private final Source m_humanPlayer;
    private final SimWorld m_world;
    private final boolean m_isBlue;
    private final Timer m_timer;

    public SourceDefault(Source source, SimWorld world, boolean isBlue) {
        m_humanPlayer = source;
        m_world = world;
        m_isBlue = isBlue;
        m_timer = new Timer();
        addRequirements(source);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        // feed if there's a nearby friend, but not too fast
        if (nearFriend() && m_timer.hasElapsed(kPeriodS)) {
            m_humanPlayer.feed();
            m_timer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
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
                System.out.printf("ignoring %s\n", body);
                continue;
            }

            return true;
        }
        return false;
    }
}
