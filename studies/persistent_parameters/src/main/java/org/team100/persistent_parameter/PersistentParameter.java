package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj.Preferences;

/** A parameter useful for tuning. */
public class PersistentParameter {
    private final String m_key;
    private final double m_defaultValue;

    /** Use the persisted value if it exists. */
    public PersistentParameter(String key, double defaultValue) {
        m_key = key;
        m_defaultValue = defaultValue;
        Preferences.initDouble(key, defaultValue);
    }

    /** Set the value in the store and the network. */
    public void set(double value) {
        Preferences.setDouble(m_key, value);
    }

    /** Get the value most recently received (or read from storage). */
    public double get() {
        return Preferences.getDouble(m_key, m_defaultValue);
    }

    /** Set the store and the network to the default value. */
    public void reset() {
        System.out.println("reset");
        Preferences.setDouble(m_key, m_defaultValue);
    }
}
