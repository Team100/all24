package org.team100.lib.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Remembers the most recent desired heading, substituting null if there's any
 * dtheta input.
 */
public class HeadingLatch {
    private static final double unlatch = 0.1;
    private Rotation2d m_desiredRotation = null;

    public Rotation2d latchedRotation(Rotation2d pov, Twist2d input) {
        if (Math.abs(input.dtheta) > unlatch) {
            m_desiredRotation = null;
        } else if (pov != null) {
            m_desiredRotation = pov;
        }
        return m_desiredRotation;
    }

    public void unlatch() {
        m_desiredRotation = null;
    }
}
