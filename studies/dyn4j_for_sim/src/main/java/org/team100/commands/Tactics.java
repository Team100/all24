package org.team100.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.field.FieldMap;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.Heuristics;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Low level drive motion heuristics that can be used by any command. */
public class Tactics {
    // this is 1/r
    // try a little less repulsion for now
    private static final int kRobotRepulsion = 25;
    // steering around robots
    private static final int kRobotSteer = 50;
    // coefficient for 1/r
    private static final int kSubwooferRepulsion = 100;
    // constant within 1m
    private static final int kWallRepulsion = 50;
    // steering around obstacles
    private static final int kSteer = 500;
    // targets appearing to move faster than this are probably false associations.
    private static final double kMaxTargetVelocity = 4;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;

    public Tactics(DriveSubsystem drive, CameraSubsystem camera) {
        m_drive = drive;
        m_camera = camera;
    }

    /**
     * @param avoidEdges some goals are near the edge, so turn this off.
     */
    public FieldRelativeVelocity apply(boolean avoidEdges, boolean avoidRobots) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Pose2d pose = m_drive.getPose();
        FieldRelativeVelocity velocity = m_drive.getVelocity();
        v = v.plus(Tactics.avoidObstacles(pose, velocity));
        if (avoidEdges)
            v = v.plus(Tactics.avoidEdges(pose));
        v = v.plus(Tactics.avoidSubwoofers(pose));
        if (avoidRobots) {
            NavigableMap<Double, RobotSighting> recentSightings = m_camera.recentSightings();
            v = v.plus(Tactics.steerAroundRobots(pose, velocity, recentSightings));
            v = v.plus(Tactics.robotRepulsion(pose, recentSightings));
        }
        return v;
    }

    /**
     * Avoid the edges of the field.
     */
    public static FieldRelativeVelocity avoidEdges(Pose2d pose) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        if (pose.getX() < 1)
            v = v.plus(new FieldRelativeVelocity(kWallRepulsion, 0, 0));
        if (pose.getX() > 15)
            v = v.plus(new FieldRelativeVelocity(-kWallRepulsion, 0, 0));
        if (pose.getY() < 1)
            v = v.plus(new FieldRelativeVelocity(0, kWallRepulsion, 0));
        if (pose.getY() > 7)
            v = v.plus(new FieldRelativeVelocity(0, -kWallRepulsion, 0));
        return v;
    }

    /**
     * Avoid the subwoofers.
     */
    public static FieldRelativeVelocity avoidSubwoofers(Pose2d pose) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Map.Entry<String, Pose2d> entry : FieldMap.subwoofers.entrySet()) {
            Translation2d target = entry.getValue().getTranslation();
            Translation2d robotRelativeToTarget = pose.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < 3) {
                // force goes as 1/r.
                Translation2d force = robotRelativeToTarget.times(kSubwooferRepulsion / (norm * norm));
                v = v.plus(new FieldRelativeVelocity(force.getX(), force.getY(), 0));
            }
        }
        return v;
    }

    /**
     * Avoid the stage posts.
     */
    public static FieldRelativeVelocity avoidObstacles(Pose2d myPosition, FieldRelativeVelocity velocity) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
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
            v = v.plus(new FieldRelativeVelocity(force.x, force.y, 0));
        }
        return v;
    }

    /**
     * Extrapolate current course and steer to void hitting robots in the future;
     * assumes they're not moving, which is a terrible assumption.
     */
    public static FieldRelativeVelocity steerAroundRobots(
            Pose2d myPosition,
            FieldRelativeVelocity myVelocity,
            NavigableMap<Double, RobotSighting> recentSightings) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        List<Translation2d> nearby = new ArrayList<>();
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
            v = v.plus(new FieldRelativeVelocity(force.x, force.y, 0));

        }
        return v;

    }

    /** A simpler method for avoiding robots: 1/r force. */
    public static FieldRelativeVelocity robotRepulsion(Pose2d myPosition,
            NavigableMap<Double, RobotSighting> recentSightings) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        List<Translation2d> nearby = new ArrayList<>();

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
                v = v.plus(new FieldRelativeVelocity(force.getX(), force.getY(), 0));
            }

        }
        return v;
    }
}
