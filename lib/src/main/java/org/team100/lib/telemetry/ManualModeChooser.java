package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;

import org.team100.lib.commands.drivetrain.ManualMode;

public class ManualModeChooser extends NamedChooser<ManualMode.Mode> {
    private static final Map<String, ManualModeChooser> choosers = new HashMap<>();

    private ManualModeChooser(String name) {
        super(name);
    }

    public static ManualModeChooser get(String name) {
        if (choosers.containsKey(name)) {
            return choosers.get(name);
        }
        ManualModeChooser c = new ManualModeChooser(name);
        choosers.put(name, c);
        return c;
    }
}
