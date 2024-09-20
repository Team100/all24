package org.team100.lib.telemetry;

import org.team100.lib.telemetry.Telemetry.Level;

/**
 * Matches the glass "Field2d" widget, which requires a specific schema.
 * 
 * Enabled by default.
 */
public class FieldLogger extends RootLogger {

    FieldLogger(Telemetry telemetry) {
        super(telemetry, "field");
        stringLogger(Level.COMP, ".type").log(() -> "Field2d");
    }

}
