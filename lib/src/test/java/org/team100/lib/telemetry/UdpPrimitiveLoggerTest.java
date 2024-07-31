package org.team100.lib.telemetry;

import java.net.SocketException;
import java.net.UnknownHostException;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.Telemetry.Level;

class UdpPrimitiveLoggerTest {
    @Test
    void testSending() throws UnknownHostException, SocketException {
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                () -> true,
                new UdpPrimitiveLogger(),
                () -> false,
                null);
        for (int i = 0; i < 100; ++i) {
            logger.logBoolean(Level.COMP, "key", () -> true);
            logger.logDouble(Level.COMP, "key", () -> 100.0);
            logger.logInt(Level.COMP, "key", () -> 100);
            logger.logFloat(Level.COMP, "key", () -> (float) 100.0);
            logger.logDoubleArray(Level.COMP, "key", () -> new double[] { 1.0, 2.0 });
            logger.logDoubleObjArray(Level.COMP, "key", () -> new Double[] { 1.0, 2.0 });
            logger.logLong(Level.COMP, "key", () -> (long) 100);
            logger.logString(Level.COMP, "key", () -> "value");
        }
    }

}
