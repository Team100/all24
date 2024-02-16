package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.team100.lib.commands.simple.SimpleManualMode;

public class SimpleManualModeChooser extends NamedChooser<SimpleManualMode.Mode> {
    private static final Map<String, SimpleManualModeChooser> choosers = new ConcurrentHashMap<>();

    private SimpleManualModeChooser(String name) {
        super(name);
    }

    public static SimpleManualModeChooser get(String name) {
        if (choosers.containsKey(name)) {
            return choosers.get(name);
        }
        SimpleManualModeChooser c = new SimpleManualModeChooser(name);
        choosers.put(name, c);
        return c;
    }
}
