package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.net.StandardSocketOptions;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.function.Consumer;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.PrimitiveLogger2.IntLogger;
import org.team100.lib.telemetry.PrimitiveLogger2.StringLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveLogger2Test {

    ByteBuffer bb;

    /** Send some examples. */
    @Test
    void testSending() throws UnknownHostException, SocketException {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2((x) -> bb = x);
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
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2((x) -> bb = x);
        SupplierLogger logger = new SupplierLogger(
                Telemetry.get(),
                "root",
                () -> true,
                udpLogger,
                () -> false,
                null);

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

    @Test
    void testStringToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2((x) -> bb = x);
        StringLogger s = l.new UdpStringLogger("label");
        s.log("hello");
        l.flush();
        assertEquals(10, bb.remaining());
        byte[] b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 6, b[1]); // string type
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 5, b[4]); // length
        assertEquals((byte) 104, b[5]); // "h"
        assertEquals((byte) 101, b[6]);// "e"
        assertEquals((byte) 108, b[7]);// "l"
        assertEquals((byte) 108, b[8]);// "l"
        assertEquals((byte) 111, b[9]);// "o"

        // this should fill the buffer with the label
        l.dumpLabels();
        assertEquals(11, bb.remaining());
        b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 7, b[1]); // label type
        assertEquals((byte) 0, b[2]); // offset
        assertEquals((byte) 0, b[3]); // offset
        assertEquals((byte) 1, b[4]); // number of labels
        assertEquals((byte) 5, b[5]); // length
        assertEquals((byte) 108, b[6]);// "l"
        assertEquals((byte) 97, b[7]);// "a"
        assertEquals((byte) 98, b[8]);// "b"
        assertEquals((byte) 101, b[9]);// "e"
        assertEquals((byte) 108, b[10]);// "l"
    }

    @Test
    void testMultiToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2((x) -> bb = x);
        StringLogger s = l.new UdpStringLogger("label");
        s.log("hello");
        IntLogger i = l.new UdpIntLogger("foo");
        i.log(1);
        l.flush();
        assertEquals(18, bb.remaining());
        byte[] b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 3, b[1]); // int type
        assertEquals((byte) 0, b[2]); // key MSB
        assertEquals((byte) 17, b[3]); // key LSB
        assertEquals((byte) 0, b[4]); // int value
        assertEquals((byte) 0, b[5]); // int value
        assertEquals((byte) 0, b[6]); // int value
        assertEquals((byte) 1, b[7]); // int value
        assertEquals((byte) 0, b[8]);
        assertEquals((byte) 6, b[9]); // string type
        assertEquals((byte) 0, b[10]); // key high byte
        assertEquals((byte) 16, b[11]); // key low byte
        assertEquals((byte) 5, b[12]); // length
        assertEquals((byte) 104, b[13]); // "h"
        assertEquals((byte) 101, b[14]);// "e"
        assertEquals((byte) 108, b[15]);// "l"
        assertEquals((byte) 108, b[16]);// "l"
        assertEquals((byte) 111, b[17]);// "o"

        // this should fill the buffer with the label
        l.dumpLabels();
        assertEquals(15, bb.remaining());
        b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 7, b[1]); // label type
        assertEquals((byte) 0, b[2]); // offset
        assertEquals((byte) 0, b[3]); // offset
        assertEquals((byte) 2, b[4]); // number of labels
        assertEquals((byte) 5, b[5]); // length
        assertEquals((byte) 108, b[6]);// "l"
        assertEquals((byte) 97, b[7]);// "a"
        assertEquals((byte) 98, b[8]);// "b"
        assertEquals((byte) 101, b[9]);// "e"
        assertEquals((byte) 108, b[10]);// "l"
        assertEquals((byte) 3, b[11]);// length
        assertEquals((byte) 102, b[12]);// "f"
        assertEquals((byte) 111, b[13]);// "o"
        assertEquals((byte) 111, b[14]);// "o"
    }
}
