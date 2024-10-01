package org.team100.lib.logging;

import org.team100.lib.util.NamedChooser;

public class LogLevelChooser extends NamedChooser<Level> {
    private static final String kName = "Log Level";
    private static final LogLevelChooser instance = new LogLevelChooser(kName);

    private LogLevelChooser(String name) {
        super(name);
    }

    public static LogLevelChooser get() {
        return instance;
    }
}
