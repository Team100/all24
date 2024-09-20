package org.team100.lib.telemetry;

import org.team100.lib.logging.SupplierLogger;

public class NetworkLogger extends SupplierLogger {
    NetworkLogger(Telemetry telemetry, String root) {
        super(telemetry, root, telemetry.udpLogger);
    }
}
