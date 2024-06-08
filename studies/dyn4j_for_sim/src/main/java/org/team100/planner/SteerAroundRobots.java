package org.team100.planner;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Extrapolate desired course and steer to void hitting robots in the future.
 * Assumes they're not moving, which is a terrible assumption.
 */
public class SteerAroundRobots implements Tactic {
    private static final double kRobotSteer = 8;
    private static final double kMaxTargetVelocity = 4;

    private final DriveSubsystem m_drive;
    private final CameraSubsystem m_camera;
    private final ForceViz m_viz;
    private final Heuristics m_heuristics;
    private final boolean m_debug;

    /**
     * @param drive  provides pose
     * @param camera provides robot sightings
     */
    public SteerAroundRobots(
            DriveSubsystem drive,
            CameraSubsystem camera,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_camera = camera;
        m_viz = viz;
        m_heuristics = new Heuristics(debug);
        m_debug = debug && Debug.enable();
    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity myVelocity) {
        Pose2d myPosition = m_drive.getPose();
        NavigableMap<Double, RobotSighting> recentSightings = m_camera.recentSightings();
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
            Vector2 steer = m_heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(myVelocity.x(), myVelocity.y()),
                    new Vector2(mostRecentPosition.getX(), mostRecentPosition.getY()),
                    1.0);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kRobotSteer);
            if (m_debug)
                System.out.printf(" steerAroundRobots target (%5.2f, %5.2f) F (%5.2f, %5.2f)",
                        mostRecentPosition.getX(), mostRecentPosition.getY(), force.x, force.y);
            FieldRelativeVelocity robotSteer = new FieldRelativeVelocity(force.x, force.y, 0);
            if (m_debug)
                m_viz.tactics(myPosition, robotSteer);
            v = v.plus(robotSteer);
        }
        return v;
    }
}
