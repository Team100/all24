package org.team100.lib.planner;

import java.util.function.Supplier;

import org.team100.lib.field.FieldMap2024;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Steer to avoid the stage posts.
 */
public class SteerAroundObstacles implements Tactic {
    private static final double kObstacleSteer = 1;

    private final Supplier<Pose2d> m_drive;
    private final ForceViz m_viz;
    private final Heuristics m_heuristics;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public SteerAroundObstacles(
            Supplier<Pose2d> drive,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_viz = viz;
        m_heuristics = new Heuristics(debug);
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity velocity) {
        Pose2d myPosition = m_drive.get();
        // only look at obstacles less than 1 second away.
        final double maxDistance = velocity.norm();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Pose2d pose : FieldMap2024.stagePosts.values()) {
            Translation2d obstacleLocation = pose.getTranslation();
            double distance = myPosition.getTranslation().getDistance(obstacleLocation);
            if (distance > maxDistance) // ignore far-away obstacles
                continue;
            FieldRelativeVelocity steer = m_heuristics.steerToAvoid(
                    myPosition.getTranslation(),
                    velocity,
                    obstacleLocation,
                    1.0);
            if (steer.norm() < 1e-3)
                continue;
            FieldRelativeVelocity force = steer.times(kObstacleSteer);
            if (m_debug)
                System.out.printf(" steerAroundObstacles target (%5.2f, %5.2f) F (%5.2f, %5.2f)",
                        obstacleLocation.getX(),
                        obstacleLocation.getY(),
                        force.x(),
                        force.y());
            FieldRelativeVelocity steering = new FieldRelativeVelocity(force.x(), force.y(), 0);
            if (m_debug)
                m_viz.tactics(pose.getTranslation(), steering);
            v = v.plus(steering);
        }
        return v;
    }
}
