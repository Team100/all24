package org.team100.lib.motion.drivetrain.manual;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Transform manual input into a field-relative Twist2d.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements Supplier<Twist2d> {
    private static final double kScale = 1.0;
    private final Supplier<Twist2d> m_input;

    public ManualFieldRelativeSpeeds(Supplier<Twist2d> input) {
        m_input = input;
    }

    @Override
    public Twist2d get() {
        Twist2d input = m_input.get();
        return new Twist2d(
                kScale * input.dx,
                kScale * input.dy,
                kScale * input.dtheta);
    }

}
