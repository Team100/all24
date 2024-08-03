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
            logger.logBoolean(Level.COMP, "boolkey", () -> true);
            logger.logDouble(Level.COMP, "doublekey", () -> 100.0);
            logger.logInt(Level.COMP, "intkey", () -> 100);
            logger.logDoubleArray(Level.COMP, "doublearraykey", () -> new double[] { 1.0, 2.0 });
            logger.logDoubleObjArray(Level.COMP, "doubleobjarraykey", () -> new Double[] { 1.0, 2.0 });
            logger.logLong(Level.COMP, "longkey", () -> (long) 100);
            logger.logString(Level.COMP, "stringkey", () -> "value");
        }
    }

}
