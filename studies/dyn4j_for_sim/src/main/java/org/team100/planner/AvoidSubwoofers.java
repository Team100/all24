package org.team100.planner;

import java.util.Map;

import org.team100.Debug;
import org.team100.field.FieldMap;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.sim.ForceViz;
import org.team100.subsystems.DriveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Avoid the subwoofers.
 */
public class AvoidSubwoofers implements Tactic {
    private static final double kSubwooferRepulsion = 5;

    private final DriveSubsystem m_drive;
    private final boolean m_debug;

    /**
     * @param drive provides pose
     */
    public AvoidSubwoofers(DriveSubsystem drive, boolean debug) {
        m_drive = drive;
        m_debug = debug && Debug.enable();

    }

    @Override
    public FieldRelativeVelocity apply(FieldRelativeVelocity desired) {
        Pose2d pose = m_drive.getPose();
        final double maxDistance = 3;
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        for (Map.Entry<String, Pose2d> entry : FieldMap.subwoofers.entrySet()) {
            Translation2d target = entry.getValue().getTranslation();
            Translation2d robotRelativeToTarget = pose.getTranslation().minus(target);
            double norm = robotRelativeToTarget.getNorm();
            if (norm < maxDistance) {
                // unit vector in the direction of the force
                Translation2d normalized = robotRelativeToTarget.div(norm);
                // scale the force so that it's zero at the maximum distance, i.e. C0 smooth.
                double scale = kSubwooferRepulsion * (1 / norm - 1 / maxDistance);
                Translation2d force = normalized.times(scale);
                if (m_debug)
                    System.out.printf(" avoidSubwoofers (%5.2f, %5.2f)", force.getX(), force.getY());
                FieldRelativeVelocity subwooferRepel = new FieldRelativeVelocity(force.getX(), force.getY(), 0);
                if (m_debug)
                    ForceViz.put("tactics", pose, subwooferRepel);
                v = v.plus(subwooferRepel);
            }
        }
        return v;

    }

}
