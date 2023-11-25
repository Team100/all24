package org.team100.lib.config;

import java.util.HashMap;
import java.util.Map;
import java.util.function.IntSupplier;

public class BatteryFactory {
    private final Map<Integer, Battery> batteries = new HashMap<>();

    private final IntSupplier m_id;

    public BatteryFactory(IntSupplier id) {
        m_id = id;
        for (Battery b : Battery.values()) {
            batteries.put(b.id, b);
        }
    }

    public Battery get() {
        int id = m_id.getAsInt();
        if (batteries.containsKey(id))
            return batteries.get(id);
        return Battery.UNKNOWN;
    }
}
