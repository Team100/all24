package org.team100.commands;

import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.robot.RobotSubsystem;
import org.team100.robot.RobotSubsystem.RobotSighting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Stay between opponents and their source.
 * 
 * Never finishes.
 */
public class DefendSource extends Command {
    private final Alliance m_alliance;
    private final RobotSubsystem m_robot;
    private final Tactics m_tactics;

    public DefendSource(Alliance alliance, RobotSubsystem robot) {
        m_alliance = alliance;
        m_robot = robot;
        m_tactics = new Tactics(robot);
        addRequirements(robot);
    }

    @Override
    public void execute() {
        m_tactics.avoidObstacles();
        m_tactics.avoidEdges();
        m_tactics.avoidSubwoofers();
        // it's ok to collide with robots
        // m_tactics.steerAroundRobots();
        // m_tactics.robotRepulsion();
        work();
    }

    /** Never finishes but might be interrupted. */
    @Override
    public void end(boolean interrupted) {
        m_alliance.onEnd(m_robot, this);
    }

    /**
     * If no robots are around, wait near the source. If there is a foe, stay
     * between it and the source. Avoid getting too close to the source.
     */
    private void work() {
        Pose2d pose = m_robot.getPose();
        Vector2 position = new Vector2(pose.getX(), pose.getY());

        // attract to the waiting spot
        Vector2 toWaitingSpot = position.to(m_robot.getRobotBody().defenderPosition());
        m_robot.getRobotBody().applyForce(toWaitingSpot.setMagnitude(200));

        // repel from the corner, and don't chase opponents, if too close
        Vector2 toCorner = position.to(m_robot.getRobotBody().opponentSourcePosition());
        if (toCorner.getMagnitude() < 2.5) {
            m_robot.getRobotBody().applyForce(toCorner.setMagnitude(-400));
            return;
        }
        // TODO: if there's an opponent nearby, stay between it and the corner.
        NavigableMap<Double, RobotSighting> recentSightings = m_robot.recentSightings();
        for (Entry<Double, RobotSighting> mostRecent : recentSightings.entrySet()) {
            RobotSighting mostRecentSighting = mostRecent.getValue();
            // don't try to defend friends
            if (mostRecentSighting.friend())
                continue;
            Translation2d mostRecentPosition = mostRecentSighting.position();

            double distance = pose.getTranslation().getDistance(mostRecentPosition);
            if (distance > 4) {
                // don't react to far-away obstacles
                continue;
            }
            double fieldDistance = m_robot.getRobotBody().defenderPosition().distance(
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()));
            if (fieldDistance > 3) {
                // don't chase it too far
                continue;
            }
            // for now just drive hard towards it
            // TODO: figure out which side to push on
            Vector2 toOpponent = position.to(
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()));
            m_robot.getRobotBody().applyForce(toOpponent.setMagnitude(400));
            break;
        }

    }

}
