package org.team100.lib.telemetry;

import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Matches the glass "Field2d" widget, which requires a specific schema.
 * 
 * Enabled by default.
 */
public class FieldLogger extends RootLogger {

    FieldLogger(
            Telemetry telemetry,
            boolean defaultEnabledNT,
            boolean defaultEnabledUSB) {
        super(telemetry, "field", defaultEnabledNT, defaultEnabledUSB);
        logString(Level.COMP, ".type", () -> "Field2d");
    }

}
