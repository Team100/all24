package org.team100.lib.planner;

import java.util.function.Supplier;

import org.team100.lib.field.PracticeField;
import org.team100.lib.geometry.Ellipse2d;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

// TODO: 2025 version
// import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Avoid fixed obstacles.
 */
public class ObstacleRepulsion implements Tactic {
    private static final double kObstacleRepulsion = 10;

    private final Supplier<Pose2d> m_poseSupplier;
    private final ForceViz m_viz;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public ObstacleRepulsion(
            Supplier<Pose2d> poseSupplier,
            ForceViz viz,
            boolean debug) {
        m_poseSupplier = poseSupplier;
        m_viz = viz;
        m_debug = debug;
    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired) {
        Pose2d myPosition = m_poseSupplier.get();
        final double maxDistance = 1.5;
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Ellipse2d obstacle : PracticeField.obstacles) {
            Translation2d target = obstacle.findNearestPoint(myPosition.getTranslation());
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
                if (m_debug)
                    System.out.printf(" obstacleRepulsion (%5.2f, %5.2f)", force.getX(), force.getY());
                FieldRelativeVelocity repel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (m_debug)
                    m_viz.tactics(target, repel);
                v = v.plus(repel);
            }
        }
        return v;
    }
}