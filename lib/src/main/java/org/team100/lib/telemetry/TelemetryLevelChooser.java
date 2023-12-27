package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;

public class TelemetryLevelChooser extends NamedChooser<Telemetry.Level> {
    private static final Map<String, TelemetryLevelChooser> choosers = new HashMap<>();

    private TelemetryLevelChooser(String name) {
        super(name);
    }

    public static TelemetryLevelChooser get(String name) {
        if (choosers.containsKey(name)) {
            return choosers.get(name);
        }
        TelemetryLevelChooser c = new TelemetryLevelChooser(name);
        choosers.put(name, c);
        return c;
    }
}
