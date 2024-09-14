package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.PrimitiveLogger2.IntLogger;
import org.team100.lib.telemetry.PrimitiveLogger2.StringLogger;
import org.team100.lib.telemetry.SupplierLogger2.BooleanSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleArraySupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleObjArraySupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.IntSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.LongSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.StringSupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveLogger2Test {

    ByteBuffer bb;

    /** Send some examples. */
    @Test
    void testSendingLocally() {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(x -> bb = x);
        SupplierLogger2 logger = new SupplierLogger2(Telemetry.get(), "root", udpLogger);
        BooleanSupplierLogger booleanLogger = logger.booleanLogger(Level.COMP, "boolkey");
        DoubleSupplierLogger doubleLogger = logger.doubleLogger(Level.COMP, "doublekey");
        IntSupplierLogger intLogger = logger.intLogger(Level.COMP, "intkey");
        DoubleArraySupplierLogger doubleArrayLogger = logger.doubleArrayLogger(Level.COMP, "doublearraykey");
        DoubleObjArraySupplierLogger doubleObjArrayLogger = logger.doubleObjArrayLogger(Level.COMP,
                "doubleobjarraykey");
        LongSupplierLogger longLogger = logger.longLogger(Level.COMP, "longkey");
        StringSupplierLogger stringLogger = logger.stringLogger(Level.COMP, "stringkey");

        for (int i = 0; i < 100; ++i) {
            booleanLogger.log(() -> true);
            doubleLogger.log(() -> 100.0);
            intLogger.log(() -> 100);
            doubleArrayLogger.log(() -> new double[] { 1.0, 2.0 });
            doubleObjArrayLogger.log(() -> new Double[] { 1.0, 2.0 });
            longLogger.log(() -> (long) 100);
            stringLogger.log(() -> "value");
        }
        udpLogger.flush();
        assertEquals(7, udpLogger.labels.size());
        assertEquals("/root/boolkey", udpLogger.labels.get(0));
        assertEquals("/root/doublekey", udpLogger.labels.get(1));
        assertEquals("/root/intkey", udpLogger.labels.get(2));
        assertEquals("/root/doublearraykey", udpLogger.labels.get(3));
        assertEquals("/root/doubleobjarraykey", udpLogger.labels.get(4));
        assertEquals("/root/longkey", udpLogger.labels.get(5));
        assertEquals("/root/stringkey", udpLogger.labels.get(6));
    }

    @Test
    void testSendingViaUDP() {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(new UdpSender());
        SupplierLogger2 logger = new SupplierLogger2(Telemetry.get(), "root", udpLogger);
        BooleanSupplierLogger booleanLogger = logger.booleanLogger(Level.COMP, "boolkey");
        DoubleSupplierLogger doubleLogger = logger.doubleLogger(Level.COMP, "doublekey");
        IntSupplierLogger intLogger = logger.intLogger(Level.COMP, "intkey");
        DoubleArraySupplierLogger doubleArrayLogger = logger.doubleArrayLogger(Level.COMP, "doublearraykey");
        DoubleObjArraySupplierLogger doubleObjArrayLogger = logger.doubleObjArrayLogger(Level.COMP,
                "doubleobjarraykey");
        LongSupplierLogger longLogger = logger.longLogger(Level.COMP, "longkey");
        StringSupplierLogger stringLogger = logger.stringLogger(Level.COMP, "stringkey");

        for (int i = 0; i < 100; ++i) {
            booleanLogger.log(() -> true);
            doubleLogger.log(() -> 100.0);
            intLogger.log(() -> 100);
            doubleArrayLogger.log(() -> new double[] { 1.0, 2.0 });
            doubleObjArrayLogger.log(() -> new Double[] { 1.0, 2.0 });
            longLogger.log(() -> (long) 100);
            stringLogger.log(() -> "value");
        }
        udpLogger.flush();
        assertEquals(7, udpLogger.labels.size());
        assertEquals("/root/boolkey", udpLogger.labels.get(0));
        assertEquals("/root/doublekey", udpLogger.labels.get(1));
        assertEquals("/root/intkey", udpLogger.labels.get(2));
        assertEquals("/root/doublearraykey", udpLogger.labels.get(3));
        assertEquals("/root/doubleobjarraykey", udpLogger.labels.get(4));
        assertEquals("/root/longkey", udpLogger.labels.get(5));
        assertEquals("/root/stringkey", udpLogger.labels.get(6));
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
    void testAWholeLotLocally() throws InterruptedException {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(x -> bb = x);
        SupplierLogger2 logger = new SupplierLogger2(
                Telemetry.get(),
                "root",
                udpLogger);

        double t0 = Timer.getFPGATimestamp();
        final double interval = 0.02;
        final double total_time = 2;
        final int keys = 5000;
        final double expected_keys_per_sec = keys / interval;
        DoubleSupplierLogger[] loggers = new DoubleSupplierLogger[keys];
        for (int j = 0; j < keys; ++j) {
            loggers[j] = logger.doubleLogger(Level.COMP, "doublekey" + j);
        }
        System.out.println("expected keys per second: " + expected_keys_per_sec);
        for (int i = 0; i < (total_time / interval); ++i) {
            double d = Timer.getFPGATimestamp() - t0;
            double dt = interval - (d % interval);
            Thread.sleep((long) (dt * 1000) + 1);
            double t1 = Timer.getFPGATimestamp();
            for (int j = 0; j < keys; ++j) {
                double val = Math.sin(j + 0.01 * i);
                loggers[j].log(() -> val);
            }
            udpLogger.flush();
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("et %.3f\n", t2 - t1);
        }
    }

    @Test
    void testAWholeLotViaUDP() throws InterruptedException {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(new UdpSender());
        SupplierLogger2 logger = new SupplierLogger2(
                Telemetry.get(),
                "root",
                udpLogger);

        double t0 = Timer.getFPGATimestamp();
        final double interval = 0.02;
        final double total_time = 2;
        final int keys = 5000;
        final double expected_keys_per_sec = keys / interval;
        DoubleSupplierLogger[] loggers = new DoubleSupplierLogger[keys];
        for (int j = 0; j < keys; ++j) {
            loggers[j] = logger.doubleLogger(Level.COMP, "doublekey" + j);
        }
        System.out.println("expected keys per second: " + expected_keys_per_sec);
        for (int i = 0; i < (total_time / interval); ++i) {
            double d = Timer.getFPGATimestamp() - t0;
            double dt = interval - (d % interval);
            Thread.sleep((long) (dt * 1000) + 1);
            double t1 = Timer.getFPGATimestamp();
            for (int j = 0; j < keys; ++j) {
                double val = Math.sin(j + 0.01 * i);
                loggers[j].log(() -> val);
            }
            udpLogger.flush();
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("et %.3f\n", t2 - t1);
        }
    }

    @Test
    void testStringToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2((x) -> bb = x);
        StringLogger s = l.stringLogger("label");
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
        StringLogger s = l.stringLogger("label");
        s.log("hello");
        IntLogger i = l.intLogger("foo");
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
