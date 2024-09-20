package org.team100.lib.telemetry;

import org.team100.lib.logging.SupplierLogger2;

public class RootLogger extends SupplierLogger2 {
    RootLogger(Telemetry telemetry, String root) {
        super(telemetry, root, telemetry.udpLogger);
    }
}