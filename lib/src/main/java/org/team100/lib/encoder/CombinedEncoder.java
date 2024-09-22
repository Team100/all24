package org.team100.lib.encoder;

import java.util.OptionalDouble;

import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.motion.RotaryMechanism;
import org.team100.lib.telemetry.Telemetry.Level;
import org.team100.lib.util.Util;

/**
 * Proxies an absolute sensor and another sensor, corrects the position of the
 * latter every time you get the value. Also falls back to the secondary if the
 * primary fails.
 * 
 * The use case is absolute + incremental encoders, in order to do outboard
 * closed-loop position control with only outboard incremental encoders --
 * RoboRIO-attached absolute encoders are the primary, and the incremental
 * encoders are the secondary.
 * 
 * The secondary is a RotaryMechanism because we want the gear reduction to be
 * applied to the underlying encoder.
 */
public class CombinedEncoder implements RotaryPositionSensor {
    private final RotaryPositionSensor m_primary;
    private final double m_authority;
    private final RotaryMechanism m_secondary;
    private final DoubleSupplierLogger2 m_log_primary;
    private final DoubleSupplierLogger2 m_log_combined;

    /**
     * 
     * @param primary
     * @param authority how to discount the primary updates: 1.0 = it's truth, 0.0 =
     *                  ignore it.
     * @param secondary
     */
    public CombinedEncoder(
            SupplierLogger2 parent,
            RotaryPositionSensor primary,
            double authority,
            RotaryMechanism secondary) {
        m_primary = primary;
        SupplierLogger2 child = parent.child(this);
        m_authority = Util.inRange(authority, 0.0, 1.0);
        m_secondary = secondary;

        OptionalDouble optPrimaryPositionRad = m_primary.getPositionRad();
        double primaryPositionRad = optPrimaryPositionRad.getAsDouble();
        double secondaryPosition = m_secondary.getPositionRad().orElse(primaryPositionRad);
        m_secondary.setEncoderPosition(primaryPositionRad);
        m_log_primary = child.doubleLogger(Level.TRACE, "Primary Rads");
        m_log_combined = child.doubleLogger(Level.TRACE, "Combined Encoder Output");
    }

    @Override
    public OptionalDouble getPositionRad() {
        // primary range is [-pi, pi]
        // OptionalDouble optPrimaryPositionRad = m_primary.getPositionRad();
        // if (optPrimaryPositionRad.isPresent()) {
        // // Adjust the secondary.
        // // note that the absolute encoder range is [-pi,pi] but the incremental
        // encoder
        // // "winds up", so it has infinite range.
        // double primaryPositionRad = optPrimaryPositionRad.getAsDouble();
        // double secondaryPosition =
        // m_secondary.getPositionRad().orElse(primaryPositionRad);
        // // the actual adjustment is closer to the secondary
        // double adjustedRad = Math100.getMinDistance(secondaryPosition,
        // primaryPositionRad);
        // double correctedPosition = m_authority * adjustedRad + (1.0 - m_authority) *
        // secondaryPosition;
        // m_secondary.setEncoderPosition(correctedPosition);
        // return OptionalDouble.of(correctedPosition);
        // }
        // Primary is broken, maybe the secondary is still working.
        return m_secondary.getPositionRad();
    }

    @Override
    public OptionalDouble getRateRad_S() {
        // an absolute position sensor isn't a very good velocity sensor, so prefer the
        // secondary.
        OptionalDouble rate = m_secondary.getVelocityRad_S();
        if (rate.isPresent()) {
            return rate;
        }
        // Secondary is broken, maybe the primary is still working.
        return m_primary.getRateRad_S();
    }

    @Override
    public void close() {
        m_primary.close();
        m_secondary.close();
    }

    public void periodic() {
        m_log_primary.log(() -> m_primary.getPositionRad().getAsDouble());
        m_log_combined.log(() -> getPositionRad().getAsDouble());

    }

}
