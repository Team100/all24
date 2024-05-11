package org.team100.planner;

import org.team100.field.FieldMap;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Avoid obstacles. */
public class ObstacleRepulsion implements Tactic {
    private static final double kObstacleRepulsion = 10;

    private final DriveSubsystem m_drive;

    /**
     * @param drive  provides pose
     */
    public ObstacleRepulsion(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired, boolean debug) {
        Pose2d myPosition = m_drive.getPose();
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
}
