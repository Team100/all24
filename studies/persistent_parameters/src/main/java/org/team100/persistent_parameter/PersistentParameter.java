package org.team100.persistent_parameter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A parameter useful for tuning.
 * 
 * Can be influenced five ways:
 * 
 * 1. load the persistent value from Preference storage
 * 2. default in code at startup if no persistent value
 * 3. update via Network Tables (e.g. glass or coprocessor)
 * 4. update via HID axes, e.g. the custom arduino knobs Joel made.
 * 5. reset to the default via a HID button
 * 
 * Updates are sent to preference storage and the network.
 * 
 * In simulation, new values don't appear, but this seems to be
 * a bug in the simulation GUI, not a real NT bug.
 */
public class PersistentParameter implements DoubleSupplier {
    /** Config for the HID features. */
    public static record HIDConfig(
            DoubleSupplier knob,
            BooleanSupplier reset) {
    }

    private final String m_key;
    private final double m_defaultValue;
    private final HIDConfig m_conf;
    private double m_knob_offset;

    /** No HID support. */
    public PersistentParameter(String key, double defaultValue) {
        this(key, defaultValue, new HIDConfig(() -> 0.0, () -> false));
    }

    /**
     * Use the persisted value if it exists.
     * 
     * @param key          for network tables and preference json file
     * @param defaultValue if no persisted value exists
     * @param conf         includes update and reset from HID
     */
    public PersistentParameter(String key, double defaultValue, HIDConfig conf) {
        m_key = key;
        m_defaultValue = defaultValue;
        m_conf = conf;
        m_knob_offset = conf.knob().getAsDouble();
        Preferences.initDouble(key, defaultValue);

    }

    /**
     * Return the default if this is a read-only instance.
     * 
     * Reset the value if the reset supplier says so.
     * 
     * Get the value most recently received (or read from storage)
     * 
     * Adjust the value according to the knob.
     * 
     * This is synchronized to guarantee the offset arithmetic is correct.
     */
    @Override
    public synchronized double getAsDouble() {
        double knobVal = m_conf.knob().getAsDouble();
        if (m_conf.reset().getAsBoolean()) {
            m_knob_offset = knobVal;
            Preferences.setDouble(m_key, m_defaultValue);
            return m_defaultValue;
        }
        double knobNet = knobVal - m_knob_offset;
        m_knob_offset = knobVal;
        double newVal = Preferences.getDouble(m_key, m_defaultValue) + knobNet;
        Preferences.setDouble(m_key, newVal);
        return newVal;
    }
}
