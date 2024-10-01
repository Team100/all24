package org.team100.lib.experiments;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;

import org.team100.lib.util.NamedChooser;

public class ExperimentChooser extends NamedChooser<BooleanSupplier> {
    private static final Map<String, ExperimentChooser> choosers = new ConcurrentHashMap<>();

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
