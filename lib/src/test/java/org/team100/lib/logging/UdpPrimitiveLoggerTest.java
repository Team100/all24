package org.team100.lib.logging;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.net.SocketException;
import java.net.UnknownHostException;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.SupplierLogger;
import org.team100.lib.telemetry.Telemetry;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveLoggerTest {
    /** Send some examples. */
    @Test
    void testSending() throws UnknownHostException, SocketException {
        UdpPrimitiveLogger udpLogger = new UdpPrimitiveLogger();
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                udpLogger);
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
        for (String k : udpLogger.handles.keySet()) {
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

    /**
     * Send at a realistic rate (50 hz), with as many keys as possible.
     * 
     * How many keys is possible?
     * 
     * About 5000, this produces 15ms per cycle of socket.send() waiting.
     * 
     * This produces a peak packet rate of almost 300k PPS, which is about the
     * maximum possible.
     * 
     * With a python receiver also on my desktop, the loss rate at 300k PPS is
     * quite low, about 1%.
     * 
     * The encoding part of flushing is not the bottleneck: these results are the
     * same if the encoding is removed.
     * 
     * DatagramSocket and DatagramChannel behave about the same.
     * 
     * socket.connect() doesn't help.
     * 
     * @throws InterruptedException
     */
    @Test
    void testAWholeLot() throws UnknownHostException, SocketException, InterruptedException {
        UdpPrimitiveLogger udpLogger = new UdpPrimitiveLogger();
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                udpLogger);

        double t0 = Timer.getFPGATimestamp();
        final double interval = 0.02;
        final double total_time = 2;
        final int keys = 5000;
        final double expected_keys_per_sec = keys / interval;
        System.out.println("expected keys per second: " + expected_keys_per_sec);
        for (int i = 0; i < (total_time / interval); ++i) {
            double d = Timer.getFPGATimestamp() - t0;
            double dt = interval - (d % interval);
            Thread.sleep((long) (dt * 1000) + 1);
            d = Timer.getFPGATimestamp() - t0;
            double t1 = Timer.getFPGATimestamp();
            for (int j = 0; j < keys; ++j) {
                double val = Math.sin(j + 0.01 * i);
                logger.logDouble(Level.COMP, "doublekey" + j, () -> val);
            }
            udpLogger.flush();
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("et %.3f\n", t2 - t1);
        }

    }

}
