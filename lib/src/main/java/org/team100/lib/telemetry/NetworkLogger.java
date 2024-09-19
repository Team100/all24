package org.team100.lib.telemetry;

public class NetworkLogger extends SupplierLogger {
    NetworkLogger(Telemetry telemetry, String root) {
        super(telemetry, root, () -> true, telemetry.udpLogger, () -> false, null);
    }
}
