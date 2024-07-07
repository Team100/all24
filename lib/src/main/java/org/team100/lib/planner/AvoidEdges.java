package org.team100.lib.planner;

import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Avoid the edges of the field.
 */
public class AvoidEdges implements Tactic {
    private static final double kWallRepulsion = 5;

    private final Supplier<Pose2d> m_poseSupplier;
    private final ForceViz m_viz;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public AvoidEdges(
            Supplier<Pose2d> poseSupplier,
            ForceViz viz,
            boolean debug) {
        m_poseSupplier = poseSupplier;
        m_viz = viz;
        m_debug = debug;

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired) {
        Translation2d translation = m_poseSupplier.get().getTranslation();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        if (translation.getX() < 1)
            v = v.plus(new FieldRelativeVelocity(kWallRepulsion, 0, 0));
        if (translation.getX() > 15)
            v = v.plus(new FieldRelativeVelocity(-kWallRepulsion, 0, 0));
        if (translation.getY() < 1)
            v = v.plus(new FieldRelativeVelocity(0, kWallRepulsion, 0));
        if (translation.getY() > 7)
            v = v.plus(new FieldRelativeVelocity(0, -kWallRepulsion, 0));
        if (m_debug)
            System.out.printf(" avoidEdges (%5.2f, %5.2f)", v.x(), v.y());
        if (m_debug)
            m_viz.tactics(translation, v);
        return v;
    }

}