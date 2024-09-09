package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.net.SocketException;
import java.net.UnknownHostException;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.Telemetry.Level;

class UdpPrimitiveLoggerTest {
    @Test
    void testSending() throws UnknownHostException, SocketException {
        UdpPrimitiveLogger udpLogger = new UdpPrimitiveLogger();
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                () -> true,
                udpLogger,
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
        udpLogger.flush();
        assertEquals(8, udpLogger.handles.size());
        for (String k:udpLogger.handles.keySet()) {
            System.out.println(k);
        }
        assertEquals(1, udpLogger.handles.get("/root/boolkey"));
        assertEquals(2, udpLogger.handles.get("/root/doublekey"));
        assertEquals(3, udpLogger.handles.get("/root/intkey"));
        assertEquals(4, udpLogger.handles.get("/root/doublearraykey"));
        assertEquals(5, udpLogger.handles.get("/root/doubleobjarraykey"));
        assertEquals(6, udpLogger.handles.get("/root/longkey"));
        assertEquals(7, udpLogger.handles.get("/root/stringkey"));
    }

}
