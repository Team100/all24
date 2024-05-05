package org.team100.planner;

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

    /**
     * @param drive provides pose
     */
    public AvoidEdges(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired, boolean debug) {
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
        if (debug)
            System.out.printf(" avoidEdges (%5.2f, %5.2f)", v.x(), v.y());
        if (debug)
            ForceViz.put("tactics", pose, v);
        return v;
    }

}
