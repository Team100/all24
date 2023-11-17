package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Transform manual input into ChassisSpeeds.
 * 
 * The twist components, x, y, and theta, are mapped directly to the
 * corresponding ChassisSpeeds components (and scaled).
 */
public class ManualChassisSpeeds implements Supplier<ChassisSpeeds> {
    private static final double kScale = 1.0;
    private final Supplier<Twist2d> m_input;

    public ManualChassisSpeeds(Supplier<Twist2d> input) {
        m_input = input;
    }

    @Override
    public ChassisSpeeds get() {
        Twist2d input = m_input.get();
        return new ChassisSpeeds(
                kScale * input.dx,
                kScale * input.dy,
                kScale * input.dtheta);
    }
}
