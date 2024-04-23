package org.team100.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.robot.FieldMap;
import org.team100.robot.RobotSubsystem;
import org.team100.robot.RobotSubsystem.RobotSighting;
import org.team100.sim.Heuristics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Low level drive motion heuristics that can be used by any command. */
public class Tactics {
    // this is 1/r
    private static final int kRobotRepulsion = 50;
    // steering around robots
    private static final int kRobotSteer = 100;
    // coefficient for 1/r
    private static final int kSubwooferRepulsion = 100;
    // constant within 1m
    private static final int kWallRepulsion = 50;
    // steering around obstacles
    private static final int kSteer = 500;
    // targets appearing to move faster than this are probably false associations.
    private static final double kMaxTargetVelocity = 4;

    private final RobotSubsystem m_robot;

    public Tactics(RobotSubsystem robot) {
        m_robot = robot;
    }

    /**
     * Avoid the edges of the field.
     */
    public void avoidEdges() {
        Pose2d pose = m_robot.getPose();
        if (pose.getX() < 1)
            m_robot.apply(kWallRepulsion, 0, 0);
        if (pose.getX() > 15)
            m_robot.apply(-kWallRepulsion, 0, 0);
        if (pose.getY() < 1)
            m_robot.apply(0, kWallRepulsion, 0);
        if (pose.getY() > 7)
            m_robot.apply(0, -kWallRepulsion, 0);
    }

    /**
     * Avoid the subwoofers.
     */
    public void avoidSubwoofers() {
        Pose2d pose = m_robot.getPose();
        for (Map.Entry<String, Pose2d> entry : FieldMap.subwoofers.entrySet()) {
            Translation2d target = entry.getValue().getTranslation();
            Translation2d robotRelativeToTarget = pose.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < 3) {
                // force goes as 1/r.
                Translation2d force = robotRelativeToTarget.times(kSubwooferRepulsion / (norm * norm));
                m_robot.apply(force.getX(), force.getY(), 0);
            }
        }
    }

    /**
     * Avoid the stage posts.
     */
    public void avoidObstacles() {
        Pose2d myPosition = m_robot.getPose();
        FieldRelativeVelocity velocity = m_robot.getVelocity();
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d obstacleLocation = pose.getTranslation();
            double distance = myPosition.getTranslation().getDistance(obstacleLocation);
            if (distance > 4) // ignore far-away obstacles
                continue;
            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(velocity.x(), velocity.y()),
                    new Vector2(obstacleLocation.getX(), obstacleLocation.getY()), 1);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kSteer);
            m_robot.apply(force.x, force.y, 0);
        }
    }

    /**
     * Extrapolate current course and steer to void hitting robots in the future;
     * assumes they're not moving, which is a terrible assumption.
     */
    public void steerAroundRobots() {
        List<Translation2d> nearby = new ArrayList<>();
        Pose2d myPosition = m_robot.getPose();
        FieldRelativeVelocity myVelocity = m_robot.getVelocity();
        // sightings in reverse chrono order
        NavigableMap<Double, RobotSighting> recentSightings = m_robot.recentSightings();
        // look at entries in order of decreasing timestamp
        for (Entry<Double, RobotSighting> mostRecent : recentSightings.entrySet()) {
            double mostRecentTime = mostRecent.getKey();
            RobotSighting mostRecentSighting = mostRecent.getValue();
            Translation2d mostRecentPosition = mostRecentSighting.position();

            // use the rest of the map to try to find the velocity of this sight
            for (Entry<Double, RobotSighting> earlier : recentSightings.tailMap(mostRecentTime).entrySet()) {
                if (mostRecentSighting.friend() != earlier.getValue().friend()) {
                    // not same type => not the same object
                    continue;
                }
                double dt = mostRecentTime - earlier.getKey();
                Translation2d velocity = mostRecentPosition.minus(
                        earlier.getValue().position()).div(dt);
                if (velocity.getNorm() > kMaxTargetVelocity) {
                    // unrealistic velocity => not the same object
                    continue;
                }
                // TODO: do something with target velocity.
                // TODO: ignore old sightings once we have seen one of each target
            }

            if (!nearby.isEmpty()) {
                Translation2d nearest = mostRecentPosition.nearest(nearby);
                if (mostRecentPosition.getDistance(nearest) < 1) {
                    // ignore near-duplicates
                    continue;
                }
            }
            nearby.add(mostRecentPosition);

            double distance = myPosition.getTranslation().getDistance(mostRecentPosition);
            if (distance > 4) // don't react to far-away obstacles
                continue;

            // treat the target as a fixed obstacle.
            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(myVelocity.x(), myVelocity.y()),
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()), 1);

            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kRobotSteer);
            m_robot.apply(force.x, force.y, 0);
        }

    }

    /** A simpler method for avoiding robots: 1/r force. */
    public void robotRepulsion() {
        List<Translation2d> nearby = new ArrayList<>();
        Pose2d myPosition = m_robot.getPose();
        // sightings in reverse chrono order
        NavigableMap<Double, RobotSighting> recentSightings = m_robot.recentSightings();
        // look at entries in order of decreasing timestamp
        for (Entry<Double, RobotSighting> mostRecent : recentSightings.entrySet()) {
            RobotSighting mostRecentSighting = mostRecent.getValue();
            Translation2d mostRecentPosition = mostRecentSighting.position();
            if (!nearby.isEmpty()) {
                Translation2d nearest = mostRecentPosition.nearest(nearby);
                if (mostRecentPosition.getDistance(nearest) < 1) {
                    // ignore near-duplicates
                    continue;
                }
            }
            nearby.add(mostRecentPosition);

            double distance = myPosition.getTranslation().getDistance(mostRecentPosition);
            if (distance > 4) // don't react to far-away obstacles
                continue;

            Translation2d target = mostRecent.getValue().position();

            Translation2d robotRelativeToTarget = myPosition.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < 3) {
                // force goes as 1/r.
                Translation2d force = robotRelativeToTarget.times(kRobotRepulsion / (norm * norm));
                m_robot.apply(force.getX(), force.getY(), 0);
            }

        }
    }

}
