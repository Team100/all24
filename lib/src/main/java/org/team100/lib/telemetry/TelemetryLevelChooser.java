package org.team100.lib.telemetry;

public class TelemetryLevelChooser extends NamedChooser<Telemetry.Level> {
    private static final String kName = "Telemetry Level";
    private static final TelemetryLevelChooser instance = new TelemetryLevelChooser(kName);

    private TelemetryLevelChooser(String name) {
        super(name);
    }

    public static TelemetryLevelChooser get() {
        return instance;
    }
}
