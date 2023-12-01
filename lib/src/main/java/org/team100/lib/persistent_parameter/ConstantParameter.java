package org.team100.lib.persistent_parameter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Publishes the value on NT but doesn't allow change. */
public class ConstantParameter implements DoubleSupplier {
    private final double m_value;
    private final NetworkTableEntry m_entry;

    /** @param key in the Constants table
     * @param value the constant value
     */
    public ConstantParameter(String key, double value) {
        m_value = value;
        m_entry = NetworkTableInstance.getDefault().getTable("Constants").getEntry(key);
        m_entry.setDouble(value);
    }

    @Override
    public synchronized double getAsDouble() {
        // reiterate the value every time we use it.
        m_entry.setDouble(m_value);
        return m_value;
    }
}
