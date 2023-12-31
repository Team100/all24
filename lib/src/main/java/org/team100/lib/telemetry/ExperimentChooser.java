package org.team100.lib.telemetry;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class ExperimentChooser extends NamedChooser<BooleanSupplier> {
    private static final Map<String, ExperimentChooser> choosers = new HashMap<>();

    private ExperimentChooser(String name) {
        super(name);
    }

    public static ExperimentChooser get(String name) {
        if (choosers.containsKey(name)) {
            return choosers.get(name);
        }
        ExperimentChooser c = new ExperimentChooser(name);
        choosers.put(name, c);
        return c;
    }
}
