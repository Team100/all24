package org.team100.lib.motion.example1d;

import java.util.function.DoubleSupplier;

import org.team100.lib.profile.MotionProfile;
import org.team100.lib.profile.MotionState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Follows the given profile with a PIDF controller.
 * 
 * the timer starts immediately upon construction.
 */
public class PIDVelocitySupplier1d implements DoubleSupplier {
    private final MotionProfile m_profile;
    private final DoubleSupplier m_measurement;
    private final PIDController m_controller;
    private final Timer m_timer;

    public PIDVelocitySupplier1d(MotionProfile profile, DoubleSupplier measurement) {
        m_profile = profile;
        m_measurement = measurement;
        m_controller = new PIDController(1, 0, 0);
        m_timer = new Timer();
        m_timer.start();
    }

    /** @return velocity in meters per second */
    @Override
    public double getAsDouble() {
        MotionState motionState = m_profile.get(m_timer.get());
        double u_FF = motionState.getV();
        double u_FB = m_controller.calculate(m_measurement.getAsDouble(), motionState.getX());
        return u_FF + u_FB;
    }
}
