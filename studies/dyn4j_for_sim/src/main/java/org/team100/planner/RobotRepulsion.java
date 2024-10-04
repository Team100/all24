package org.team100.planner;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.team100.lib.motion.drivetrain.DriveSubsystemInterface;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.planner.ForceViz;
import org.team100.lib.planner.Tactic;
import org.team100.lib.util.Debug;
import org.team100.subsystems.CameraSubsystem;
import org.team100.subsystems.CameraSubsystem.RobotSighting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Avoid other robots.
 * Doesn't do anything if we're already heading away from the target.
 */
public class RobotRepulsion implements Tactic {
    private static final double kRobotRepulsion = 8;

    private final DriveSubsystemInterface m_drive;
    private final CameraSubsystem m_camera;
    private final ForceViz m_viz;
    private final boolean m_debug;

    /**
     * @param drive  provides pose
     * @param camera provides robot sightings
     */
    public RobotRepulsion(
            DriveSubsystemInterface drive,
            CameraSubsystem camera,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_camera = camera;
        m_viz = viz;
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity myVelocity) {
        Pose2d myPosition = m_drive.getPose();
        NavigableMap<Double, RobotSighting> recentSightings = m_camera.recentSightings();
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
                if (m_debug)
                    System.out.printf(" robotRepulsion target (%5.2f, %5.2f) range %5.2f F (%5.2f, %5.2f)",
                            target.getX(), target.getY(), norm, force.getX(), force.getY());
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
