package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Proxies an absolute "primary" sensor and another "secondary" sensor (e.g. an
 * integrated incremental motor sensor).
 * 
 * Corrects the secondary one time in the constructor, i.e. "zeros" it.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary. Note in this case the "primary" absolute
 * measurement is [-pi,pi] but the "secondary" measurement winds up to
 * [-inf,inf].
 * 
 * The secondary is a RotaryMechanism, instead of an encoder, because we want
 * the *gear reduction* to be applied to the underlying encoder.
 */
public class CombinedEncoder implements RotaryPositionSensor {
    private final RotaryPositionSensor m_primary;
    private final RotaryMechanism m_secondary;
    private final DoubleSupplierLogger2 m_log_primary;
    private final DoubleSupplierLogger2 m_log_secondary;
    private final DoubleSupplierLogger2 m_log_combined;

    /**
     * "Zeros" the secondary.
     * 
     * @param primary   absolute sensor wired to the RoboRIO
     * @param secondary incremental sensor that needs to be "zeroed"
     */
    public CombinedEncoder(
            SupplierLogger2 parent,
            RotaryPositionSensor primary,
            RotaryMechanism secondary) {
        SupplierLogger2 child = parent.child(this);
        m_primary = primary;
        m_secondary = secondary;
        // set the secondary to the "correct" value
        m_secondary.setEncoderPosition(m_primary.getPositionRad().getAsDouble());
        m_log_primary = child.doubleLogger(Level.TRACE, "Primary Rads");
        m_log_secondary = child.doubleLogger(Level.TRACE, "Secondary Rads");
        m_log_combined = child.doubleLogger(Level.TRACE, "Combined Encoder Output");
    }

    /** The secondary (incremental motor-integrated) measurement. */
    @Override
    public OptionalDouble getPositionRad() {
        return m_secondary.getPositionRad();
    }

    /** The secondary (incremental motor-integrated) measurement */
    @Override
    public OptionalDouble getRateRad_S() {
        return m_secondary.getVelocityRad_S();
    }

    @Override
    public void close() {
        m_primary.close();
        m_secondary.close();
    }

    public void periodic() {
        m_log_primary.log(() -> m_primary.getPositionRad().getAsDouble());
        m_log_secondary.log(() -> m_secondary.getPositionRad().getAsDouble());
        m_log_combined.log(() -> getPositionRad().getAsDouble());
    }

}
