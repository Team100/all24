package org.team100.lib.telemetry;

import org.team100.lib.logging.SupplierLogger;



public class RootLogger extends SupplierLogger {
    RootLogger(
            Telemetry telemetry,
            String root,
            boolean defaultEnabledNT,
            boolean defaultEnabledUSB) {
        super(telemetry, root, telemetry.ntLogger);
    }
}