package org.team100.persistent_parameter;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Provides persistent parameters, some of which map to knobs. */
public class ParameterFactory {
    private final HIDControl m_hid;
    private final Map<String, Integer> m_knobs;
    private final Map<String, PersistentParameter> m_parameters;

    public ParameterFactory(HIDControl hid,  Map<String, Integer> knobs) {
        m_hid = hid;
        // in general the knob mapping will change depending on the
        // robot identity and console identity.
        m_knobs = knobs;
        m_parameters = new ConcurrentHashMap<>();
    }

    public PersistentParameter get(String key, double defaultValue) {
        return m_parameters.computeIfAbsent(key, k -> make(k, defaultValue));
    }

    private PersistentParameter make(String key, double defaultValue) {
        if (m_knobs.containsKey(key)) {
            return new PersistentParameter(key, defaultValue,
                    () -> m_hid.knob(m_knobs.get(key)));
        } else {
            return new PersistentParameter(key, defaultValue);
        }
    }

}
