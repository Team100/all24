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
    public static record Config(
            DoubleSupplier knob,
            BooleanSupplier reset) {
    }

    private final String m_key;
    private final double m_defaultValue;
    private final Config m_conf;
    private double m_knob_offset;

    /** Zero default, no knob or reset. */
    public PersistentParameter(String key) {
        this(key, 0.0);
    }

    /** No knob or reset. */
    public PersistentParameter(String key, double defaultValue) {
        this(key, defaultValue, new Config(() -> 0.0, () -> false));
    }

    /**
     * Use the persisted value if it exists.
     * 
     * @param key          for network tables and preference json file
     * @param defaultValue if no persisted value exists
     * @param conf         includes update and reset from HID
     */
    public PersistentParameter(String key, double defaultValue, Config conf) {
        m_key = key;
        m_defaultValue = defaultValue;
        m_conf = conf;
        m_knob_offset = conf.knob().getAsDouble();
        Preferences.initDouble(key, defaultValue);
    }

    /** Set the value in the store and the network. */
    public synchronized void set(double value) {
        Preferences.setDouble(m_key, value);
    }

    /**
     * Get the value most recently received (or read from storage), adjusted by the
     * knob. Or reset the value if the reset supplier says so. This is synchronized
     * to guarantee the offset arithmetic is correct.
     */
    @Override
    public synchronized double getAsDouble() {
        double knobVal = m_conf.knob().getAsDouble();
        if (m_conf.reset().getAsBoolean()) {
            System.out.println("reset " + m_key);
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
