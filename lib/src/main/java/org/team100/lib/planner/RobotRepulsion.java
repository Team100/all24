package org.team100.lib.planner;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.function.Supplier;

import org.team100.lib.camera.RobotSighting;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Avoid other robots.
 * Doesn't do anything if we're already heading away from the target.
 */
public class RobotRepulsion implements Tactic {
    private static final double kRobotRepulsion = 8;

    private final Supplier<Pose2d> m_drive;
    private final Supplier<NavigableMap<Double, RobotSighting>> m_camera;
    private final ForceViz m_viz;
    private final boolean m_debug;

    /**
     * @param drive  provides pose
     * @param camera provides robot sightings
     */
    public RobotRepulsion(
            Supplier<Pose2d> drive,
            Supplier<NavigableMap<Double, RobotSighting>> camera,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_camera = camera;
        m_viz = viz;
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity myVelocity) {
        Pose2d myPosition = m_drive.get();
        final double maxDistance = 3;
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        List<Translation2d> nearby = new ArrayList<>();
        // look at entries in order of decreasing timestamp
        NavigableMap<Double, RobotSighting> recentSightings = m_camera.get();
        for (RobotSighting sight : recentSightings.values()) {
            Translation2d mostRecent = sight.position();
            if (!nearby.isEmpty()) {
                Translation2d nearest = mostRecent.nearest(nearby);
                if (mostRecent.getDistance(nearest) < 1) {
                    // ignore near-duplicates
                    continue;
                }
            }
            nearby.add(mostRecent);
            double distance = myPosition.getTranslation().getDistance(mostRecent);
            if (distance > 4) // don't react to far-away obstacles
                continue;
            Translation2d robotRelativeToTarget = myPosition.getTranslation().minus(mostRecent);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                // the minimum distance is something like 0.75 or 1, so
                // the maximum force is (1.3-0.3) = 1 * k
                double scale = kRobotRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (m_debug)
                    System.out.printf(" robotRepulsion target (%5.2f, %5.2f) range %5.2f F (%5.2f, %5.2f)",
                            mostRecent.getX(), mostRecent.getY(), norm, force.getX(), force.getY());
                FieldRelativeVelocity robotRepel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (myVelocity.dot(robotRepel) < 0) {
                    // don't bother repelling if we're heading away
                    if (m_debug)
                        m_viz.tactics(myPosition.getTranslation(), robotRepel);
                    v = v.plus(robotRepel);
                }
            }
        }
        return v;
    }

}
