package org.team100.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.field.FieldMap;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.sim.Heuristics;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Low level drive motion heuristics that can be used by any command.
 * 
 * Pointwise repulsive forces are inversely proportional to distance, like
 * gravity or electrostatics in two dimensions.
 */
public class Tactics {
    // TODO: get these from kinodynamics
    private static final double kMaxVelocity = 5; // m/s
    private static final double kMaxOmega = 10; // rad/s
    private static final double kRobotRepulsion = 5;
    private static final double kRobotSteer = 20;
    private static final double kSubwooferRepulsion = 5;
    private static final double kWallRepulsion = 5;
    private static final double kObstacleSteer = 40;
    private static final double kObstacleRepulsion = 10;
    private static final double kMaxTargetVelocity = 4;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;

    public Tactics(DriveSubsystem drive, CameraSubsystem camera) {
        m_drive = drive;
        m_camera = camera;
    }

    /**
     * Output is clamped to feasible v and omega.
     * 
     * @param avoidEdges some goals are near the edge, so turn this off.
     */
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired, boolean avoidEdges, boolean avoidRobots,
            boolean debug) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Pose2d pose = m_drive.getPose();
        // FieldRelativeVelocity velocity = m_drive.getVelocity();
        v = v.plus(steerAroundObstacles(pose, desired, debug));
        v = v.plus(obstacleRepulsion(pose, debug));
        if (avoidEdges)
            v = v.plus(avoidEdges(pose, debug));
        v = v.plus(avoidSubwoofers(pose, debug));
        if (avoidRobots) {
            NavigableMap<Double, RobotSighting> recentSightings = m_camera.recentSightings();
            v = v.plus(steerAroundRobots(pose, desired, recentSightings, debug));
            v = v.plus(robotRepulsion(pose, recentSightings, debug));
        }
        v = v.clamp(kMaxVelocity, kMaxOmega);
        return v;
    }

    /**
     * Avoid the edges of the field.
     */
    public static FieldRelativeVelocity avoidEdges(Pose2d pose, boolean debug) {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        if (pose.getX() < 1)
            v = v.plus(new FieldRelativeVelocity(kWallRepulsion, 0, 0));
        if (pose.getX() > 15)
            v = v.plus(new FieldRelativeVelocity(-kWallRepulsion, 0, 0));
        if (pose.getY() < 1)
            v = v.plus(new FieldRelativeVelocity(0, kWallRepulsion, 0));
        if (pose.getY() > 7)
            v = v.plus(new FieldRelativeVelocity(0, -kWallRepulsion, 0));
        if (debug)
            System.out.printf(" avoidEdges (%5.2f, %5.2f)", v.x(), v.y());
        if (debug)
            ForceViz.put("tactics", pose, v);
        return v;
    }

    /**
     * Avoid the subwoofers.
     */
    public static FieldRelativeVelocity avoidSubwoofers(Pose2d pose, boolean debug) {
        final double maxDistance = 3;
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Map.Entry<String, Pose2d> entry : FieldMap.subwoofers.entrySet()) {
            Translation2d target = entry.getValue().getTranslation();
            Translation2d robotRelativeToTarget = pose.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                double scale = kSubwooferRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (debug)
                    System.out.printf(" avoidSubwoofers (%5.2f, %5.2f)", force.getX(), force.getY());
                FieldRelativeVelocity subwooferRepel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (debug)
                    ForceViz.put("tactics", pose, subwooferRepel);
                v = v.plus(subwooferRepel);
            }
        }
        return v;
    }

    /**
     * Steer to avoid the stage posts.
     */
    public static FieldRelativeVelocity steerAroundObstacles(
            Pose2d myPosition,
            FieldRelativeVelocity velocity,
            boolean debug) {
        // only look at obstacles less than 1 second away.
        final double maxDistance = velocity.norm();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d obstacleLocation = pose.getTranslation();
            double distance = myPosition.getTranslation().getDistance(obstacleLocation);
            if (distance > maxDistance) // ignore far-away obstacles
                continue;
            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(velocity.x(), velocity.y()),
                    new Vector2(obstacleLocation.getX(), obstacleLocation.getY()),
                    1.0);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kObstacleSteer);
            if (debug)
                System.out.printf(" steerAroundObstacles target (%5.2f, %5.2f) F (%5.2f, %5.2f)",
                        obstacleLocation.getX(),
                        obstacleLocation.getY(),
                        force.x,
                        force.y);
            FieldRelativeVelocity steering = new FieldRelativeVelocity(force.x, force.y, 0);
            if (debug)
                ForceViz.put("tactics", pose, steering);
            v = v.plus(steering);
        }
        return v;
    }

    /** Avoid obstacles. */
    public static FieldRelativeVelocity obstacleRepulsion(Pose2d myPosition, boolean debug) {
        final double maxDistance = 1.5;
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d target = pose.getTranslation();
            Translation2d robotRelativeToTarget = myPosition.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                // the minimum distance is something like 0.75, so
                // the maximum force is (1.3-0.3) = 0.6 * k
                double scale = kObstacleRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (debug)
                    System.out.printf(" obstacleRepulsion (%5.2f, %5.2f)", force.getX(), force.getY());
                FieldRelativeVelocity repel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (debug)
                    ForceViz.put("tactics", pose, repel);
                v = v.plus(repel);
            }
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
            NavigableMap<Double, RobotSighting> recentSightings,
            boolean debug) {
        // only look at robots less than 1 second away.
        final double maxDistance = myVelocity.norm();
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
            if (distance > maxDistance) // don't react to far-away obstacles
                continue;

            // treat the target as a fixed obstacle.
            Vector2 steer = Heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(myVelocity.x(), myVelocity.y()),
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()),
                    1.0);

            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kRobotSteer);
            if (debug)
                System.out.printf(" steerAroundRobots target (%5.2f, %5.2f) F (%5.2f, %5.2f)",
                        mostRecentPosition.getX(), mostRecentPosition.getY(), force.x, force.y);
            FieldRelativeVelocity robotSteer = new FieldRelativeVelocity(force.x, force.y, 0);
            if (debug)
                ForceViz.put("tactics", myPosition, robotSteer);
            v = v.plus(robotSteer);

        }
        return v;

    }

    /** Avoid other robots. */
    public static FieldRelativeVelocity robotRepulsion(
            Pose2d myPosition,
            NavigableMap<Double, RobotSighting> recentSightings,
            boolean debug) {
        final double maxDistance = 3;
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
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                // the minimum distance is something like 0.75 or 1, so
                // the maximum force is (1.3-0.3) = 1 * k
                double scale = kRobotRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (debug)
                    System.out.printf(" robotRepulsion target (%5.2f, %5.2f) range %5.2f F (%5.2f, %5.2f)",
                            target.getX(), target.getY(), norm, force.getX(), force.getY());
                FieldRelativeVelocity robotRepel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (debug)
                    ForceViz.put("tactics", myPosition, robotRepel);
                v = v.plus(robotRepel);
            }

        }
        return v;
    }
}
