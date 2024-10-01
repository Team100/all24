package org.team100.lib.logging;

import org.team100.lib.util.NamedChooser;

public class LevelChooser extends NamedChooser<Level> {
    private static final String kName = "Log Level";
    private static final LevelChooser instance = new LevelChooser(kName);

    private LevelChooser(String name) {
        super(name);
    }

    public static LevelChooser get() {
        return instance;
    }
}
