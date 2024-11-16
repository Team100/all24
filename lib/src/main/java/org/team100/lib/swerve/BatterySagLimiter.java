package org.team100.lib.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Attempts to limit acceleration based on the battery voltage, to avoid
 * browning out.
 * 
 * Above 7V, there is no limit. Maybe this should be 8V?
 * At 6V, the allowed acceleration should be zero.
 * 
 * We set the brownout voltage to 5.5 V in {@link Robot}.
 */
public class BatterySagLimiter {

    private final DoubleSupplier m_voltage;
    private final InterpolatingDoubleTreeMap m_table;

    public BatterySagLimiter(DoubleSupplier voltage) {
        // there's a supplier here so that the tests don't need to use the
        // RobotController HAL, which sometimes mysteriously fails.
        m_voltage = voltage;
        m_table = new InterpolatingDoubleTreeMap();
        m_table.put(7.0, 1.0);
        m_table.put(6.0, 0.0);
    }

    public double get() {
        return m_table.get(m_voltage.getAsDouble());
    }

}
