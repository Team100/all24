package org.team100.planner;

import org.dyn4j.geometry.Vector2;
import org.team100.Debug;
import org.team100.field.FieldMap;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Steer to avoid the stage posts.
 */
public class SteerAroundObstacles implements Tactic {
    private static final double kObstacleSteer = 5;

    private final DriveSubsystem m_drive;
    private final Heuristics m_heuristics;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public SteerAroundObstacles(DriveSubsystem drive, boolean debug) {
        m_drive = drive;
        m_heuristics = new Heuristics(debug);
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity velocity) {
        Pose2d myPosition = m_drive.getPose();
        // only look at obstacles less than 1 second away.
        final double maxDistance = velocity.norm();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Pose2d pose : FieldMap.stagePosts.values()) {
            Translation2d obstacleLocation = pose.getTranslation();
            double distance = myPosition.getTranslation().getDistance(obstacleLocation);
            if (distance > maxDistance) // ignore far-away obstacles
                continue;
            Vector2 steer = m_heuristics.steerToAvoid(
                    new Vector2(myPosition.getX(), myPosition.getY()),
                    new Vector2(velocity.x(), velocity.y()),
                    new Vector2(obstacleLocation.getX(), obstacleLocation.getY()),
                    1.0);
            if (steer.getMagnitude() < 1e-3)
                continue;
            Vector2 force = steer.product(kObstacleSteer);
            if (m_debug)
                System.out.printf(" steerAroundObstacles target (%5.2f, %5.2f) F (%5.2f, %5.2f)",
                        obstacleLocation.getX(),
                        obstacleLocation.getY(),
                        force.x,
                        force.y);
            FieldRelativeVelocity steering = new FieldRelativeVelocity(force.x, force.y, 0);
            if (m_debug)
                ForceViz.put("tactics", pose, steering);
            v = v.plus(steering);
        }
        return v;
    }
}
