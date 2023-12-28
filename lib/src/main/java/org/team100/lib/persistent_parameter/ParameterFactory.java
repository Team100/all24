package org.team100.lib.persistent_parameter;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.DoubleSupplier;

/**
 * Provides persistent parameters, some of which map to knobs.
 */
public class ParameterFactory {
    private final Map<String, PersistentParameter.HIDConfig> m_knobs;
    private final Map<String, DoubleSupplier> m_parameters;

    public ParameterFactory(Map<String, PersistentParameter.HIDConfig> knobs) {
        m_knobs = knobs;
        m_parameters = new ConcurrentHashMap<>();
    }

    /**
     * Return a parameter linked to a preference, with knob support
     * if configured.
     * 
     * @param key          in the Preferences table
     * @param defaultValue on reset or if new
     */
    public DoubleSupplier mutable(String key, double defaultValue) {
        return m_parameters.computeIfAbsent(key, k -> make(k, defaultValue));
    }

    /**
     * Return a constant parameter. It's just a constant that's published to Network
     * Tables.
     */
    public DoubleSupplier constant(String key, double value) {
        return m_parameters.computeIfAbsent(key, k -> new ConstantParameter(k, value));
    }

    private DoubleSupplier make(String key, double defaultValue) {
        if (m_knobs.containsKey(key)) {
            PersistentParameter.HIDConfig conf = m_knobs.get(key);
            return new PersistentParameter(key, defaultValue, conf);
        } else {
            return new PersistentParameter(key, defaultValue);
        }
    }
}
