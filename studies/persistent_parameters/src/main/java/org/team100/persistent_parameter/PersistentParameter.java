package org.team100.persistent_parameter;

import edu.wpi.first.wpilibj.Preferences;

public class PersistentParameter {
    private final String m_key;
    private final double m_defaultValue;

    public PersistentParameter(String key, double defaultValue) {
        m_key = key;
        m_defaultValue = defaultValue;
        Preferences.initDouble(key, defaultValue);
    }

    public void set(double value) {
        Preferences.setDouble(m_key, value);
    }

    public double get() {
        return Preferences.getDouble(m_key, m_defaultValue);
    }
}
