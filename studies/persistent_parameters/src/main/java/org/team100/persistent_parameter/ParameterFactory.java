package org.team100.persistent_parameter;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Provides persistent parameters, some of which map to knobs. */
public class ParameterFactory {
    private final HIDControl m_hid;
    private final Map<String, Integer> m_knobs;
    private final Map<String, PersistentParameter> m_parameters;

    public ParameterFactory(HIDControl hid) {
        m_hid = hid;
        // in general the knob mapping will change depending on the
        // robot identity and console identity.
        m_knobs = new HashMap<>();
        m_parameters = new ConcurrentHashMap<>();
        // keep these in order to match the console labels
        m_knobs.put("foo", 0);
        m_knobs.put("bar", 1);
        m_knobs.put("baz", 2);
        m_knobs.put("biz", 3);
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
