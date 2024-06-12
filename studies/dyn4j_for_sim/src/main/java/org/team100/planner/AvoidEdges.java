package org.team100.planner;

import org.team100.Debug;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Avoid the edges of the field.
 */
public class AvoidEdges implements Tactic {
    private static final double kWallRepulsion = 5;

    private final DriveSubsystem m_drive;
    private final ForceViz m_viz;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public AvoidEdges(
            DriveSubsystem drive,
            ForceViz viz,
            boolean debug) {
        m_drive = drive;
        m_viz = viz;
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired) {
        Pose2d pose = m_drive.getPose();
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        if (pose.getX() < 1)
            v = v.plus(new FieldRelativeVelocity(kWallRepulsion, 0, 0));
        if (pose.getX() > 15)
            v = v.plus(new FieldRelativeVelocity(-kWallRepulsion, 0, 0));
        if (pose.getY() < 1)
            v = v.plus(new FieldRelativeVelocity(0, kWallRepulsion, 0));
        if (pose.getY() > 7)
            v = v.plus(new FieldRelativeVelocity(0, -kWallRepulsion, 0));
        if (m_debug)
            System.out.printf(" avoidEdges (%5.2f, %5.2f)", v.x(), v.y());
        if (m_debug)
            m_viz.tactics(pose, v);
        return v;
    }

}
