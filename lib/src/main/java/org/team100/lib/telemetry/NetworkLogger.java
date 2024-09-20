package org.team100.lib.telemetry;

import org.team100.lib.logging.SupplierLogger2;

public class NetworkLogger extends SupplierLogger2 {
    NetworkLogger(Telemetry telemetry, String root) {
        super(telemetry, root, telemetry.udpLogger);
    }
}
