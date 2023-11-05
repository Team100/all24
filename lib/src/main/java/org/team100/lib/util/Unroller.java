package org.team100.lib.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Wrap any supplier of rotations to remove discontinuity, so that the
 * Kalman Filter will grok them. Is there a way for the KF to understand
 * wrapping?
 * 
 * Maintains the orientation (NED or NWU) of the supplier.
 */
public class Unroller implements Supplier<Rotation2d> {
    private final Supplier<Rotation2d> m_goniometer;
    private final EdgeCounter m_counter;

    public Unroller(Supplier<Rotation2d> goniometer) {
        m_goniometer = goniometer;
        m_counter = new EdgeCounter(-0.5 * Math.PI, 0.5 * Math.PI);
    }

    @Override
    public Rotation2d get() {
        double valueRadians = m_goniometer.get().getRadians();
        int turns = m_counter.update(valueRadians);
        return new Rotation2d(valueRadians + (2 * Math.PI * turns));
    }
}
