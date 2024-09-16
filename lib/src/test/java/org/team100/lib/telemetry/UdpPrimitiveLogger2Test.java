package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.ByteBuffer;
import java.util.HexFormat;

import org.junit.jupiter.api.Test;
import org.team100.lib.telemetry.PrimitiveLogger2.IntLogger;
import org.team100.lib.telemetry.PrimitiveLogger2.StringLogger;
import org.team100.lib.telemetry.PrimitiveLogger2.BooleanLogger;
import org.team100.lib.telemetry.SupplierLogger2.BooleanSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleArraySupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleObjArraySupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.DoubleSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.IntSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.LongSupplierLogger;
import org.team100.lib.telemetry.SupplierLogger2.StringSupplierLogger;
import org.team100.lib.telemetry.Telemetry.Level;
import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveLogger2Test {

    ByteBuffer bb; // data
    ByteBuffer mb; // metadata

    /** Send some examples. */
    @Test
    void testSendingLocally() {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(x -> bb = x, x -> mb = x);
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
        assertEquals(90, bb.remaining());

        HexFormat hex = HexFormat.of();
        String expectedStr = "0000000000000000" // timestamp
                + "0010" // key = 16
                + "01" // type = boolean
                + "01" // value = true
                + "0011" // key = 17
                + "02" // type = double
                + "4059000000000000" // value
                + "0012" // key = 18
                + "03" // type = int
                + "00000064" // value
                + "0013"// key=19
                + "04" // type = double array
                + "02" // length = 2
                + "3ff0000000000000" // value
                + "4000000000000000"// value
                + "0014" // key = 20
                + "04" // type = double array
                + "02" // length = 2
                + "3ff0000000000000" // value
                + "4000000000000000"// value
                + "0015" // key = 21
                + "05" // type = long
                + "0000000000000064" // value
                + "0016" // key = 22
                + "06" // type = string
                + "05" // length = 5
                + hex.formatHex("value".getBytes());
        byte[] expectedBB = hex.parseHex(expectedStr);
        byte[] actualBB = new byte[90];
        bb.get(actualBB);
        assertArrayEquals(expectedBB, actualBB);

        assertEquals(7, udpLogger.metadata.size());
        assertEquals("/root/boolkey", udpLogger.metadata.get(0).label());
        assertEquals("/root/doublekey", udpLogger.metadata.get(1).label());
        assertEquals("/root/intkey", udpLogger.metadata.get(2).label());
        assertEquals("/root/doublearraykey", udpLogger.metadata.get(3).label());
        assertEquals("/root/doubleobjarraykey", udpLogger.metadata.get(4).label());
        assertEquals("/root/longkey", udpLogger.metadata.get(5).label());
        assertEquals("/root/stringkey", udpLogger.metadata.get(6).label());

        udpLogger.dumpLabels();
        assertEquals(147, mb.remaining());

        expectedStr = "\00\00\00\00\00\00\00\00" // timestamp
                + "\00\20" // key
                + "\01" // type
                + "\15" // 1 6 length
                + "/root/boolkey" // 13 19 label
                + "\00\21" // key 
                + "\02" // type 
                + "\17" // 1 20 length
                + "/root/doublekey" // 15 35 label
                + "\00\22" // key 
                + "\03" // type 
                + "\14" // 1 36 length
                + "/root/intkey" // 12 48 label
                + "\00\23" // key 
                + "\04" // type 
                + "\24" // 1 49 length
                + "/root/doublearraykey" // 20 69 label
                + "\00\24" // key 
                + "\04" // type 
                + "\27" // 1 70 length
                + "/root/doubleobjarraykey" // 23 93 label
                + "\00\25" // key 
                + "\05" // type 
                + "\15" // 1 94 length
                + "/root/longkey" // 13 107 label
                + "\00\26" // key 
                + "\06" // type 
                + "\17" // 1 108 length
                + "/root/stringkey"; // 15 123 label
        expectedBB = expectedStr.getBytes(StandardCharsets.US_ASCII);
        actualBB = new byte[147];
        mb.get(actualBB);
        assertArrayEquals(expectedBB, actualBB);
    }

    @Test
    void testSendingViaUDP() {
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(
                new UdpSender(UdpSender.kPort),
                new UdpSender(UdpSender.kmetadataPort));
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
        udpLogger.dumpLabels();
        assertEquals(7, udpLogger.metadata.size());
        assertEquals("/root/boolkey", udpLogger.metadata.get(0).label());
        assertEquals("/root/doublekey", udpLogger.metadata.get(1).label());
        assertEquals("/root/intkey", udpLogger.metadata.get(2).label());
        assertEquals("/root/doublearraykey", udpLogger.metadata.get(3).label());
        assertEquals("/root/doubleobjarraykey", udpLogger.metadata.get(4).label());
        assertEquals("/root/longkey", udpLogger.metadata.get(5).label());
        assertEquals("/root/stringkey", udpLogger.metadata.get(6).label());
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
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(x -> bb = x, x -> mb = x);
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
        UdpPrimitiveLogger2 udpLogger = new UdpPrimitiveLogger2(
                new UdpSender(UdpSender.kPort),
                new UdpSender(UdpSender.kmetadataPort));
        SupplierLogger2 logger = new SupplierLogger2(
                Telemetry.get(),
                "root",
                udpLogger);

        double t0 = Timer.getFPGATimestamp();
        final double interval = 0.02;
        final double total_time = 5;
        // on my desktop:
        // with shorter intervals, 600K keys per second is possible
        // at 50hz, 400K keys per second seems like the max
        // NT throttles at about 250k writes per second
        // glass can only grok a few thousand keys anyway
        // using full MTU packets (1500B) increases the 50hz throughput
        // to about 600K keys per second.  and the short-interval
        // peak throughput to about 800K keys/sec.
        // the java load at this rate is low, 10% of one CPU.
        final int keys = 20000;
        // final int keys = 5000;
        // final int keys = 100;
        final double expected_keys_per_sec = keys / interval;
        DoubleSupplierLogger[] loggers = new DoubleSupplierLogger[keys];
        for (int j = 0; j < keys; ++j) {
            loggers[j] = logger.doubleLogger(Level.COMP, "doublekey" + j);
        }
        udpLogger.sendAllLabels();
        System.out.println("expected keys per second: " + expected_keys_per_sec);
        double iterations = total_time / interval;
        // int iterations = 1;
        for (int i = 0; i < iterations; ++i) {
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
            // System.out.printf("et %.3f\n", t2 - t1);
        }
    }

    @Test
    void testStringToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2((x) -> bb = x, x -> mb = x);
        StringLogger s = l.stringLogger("label");
        s.log("hello");
        l.flush();
        assertEquals(17, bb.remaining());
        byte[] b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]); // key
        assertEquals((byte) 16, b[9]); // key
        assertEquals((byte) 6, b[10]); // type
        assertEquals((byte) 5, b[11]); // length
        assertEquals((byte) 104, b[12]); // "h"
        assertEquals((byte) 101, b[13]);// "e"
        assertEquals((byte) 108, b[14]);// "l"
        assertEquals((byte) 108, b[15]);// "l"
        assertEquals((byte) 111, b[16]);// "o"

        // this should fill the buffer with the label
        l.dumpLabels();
        assertEquals(17, mb.remaining());
        b = mb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]); // key
        assertEquals((byte) 16, b[9]); // key
        assertEquals((byte) 6, b[10]); // type
        assertEquals((byte) 5, b[11]); // length
        assertEquals((byte) 108, b[12]);// "l"
        assertEquals((byte) 97, b[13]);// "a"
        assertEquals((byte) 98, b[14]);// "b"
        assertEquals((byte) 101, b[15]);// "e"
        assertEquals((byte) 108, b[16]);// "l"
    }

    @Test
    void testBooleanToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2(x -> bb = x, x -> mb = x);
        BooleanLogger b16 = l.booleanLogger("b16");
        BooleanLogger b17 = l.booleanLogger("b17");
        b16.log(true); // overwritten
        b16.log(false);
        b17.log(true); // overwritten
        b17.log(false);
        l.flush();
        assertEquals(16, bb.remaining());
        byte[] b = bb.array();
        assertEquals((byte) 0, b[0]); // timestamp ...
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]); // key
        assertEquals((byte) 16, b[9]); // key
        assertEquals((byte) 1, b[10]); // type
        assertEquals((byte) 0, b[11]); // value
        assertEquals((byte) 0, b[12]); // key
        assertEquals((byte) 17, b[13]); // key
        assertEquals((byte) 1, b[14]); // type
        assertEquals((byte) 0, b[15]); // value

        // this should fill the buffer with the label
        l.dumpLabels();
        assertEquals(22, mb.remaining());
        b = mb.array();
        assertEquals((byte) 0, b[0]); // timestamp ...
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]);// key
        assertEquals((byte) 16, b[9]);// key
        assertEquals((byte) 1, b[10]);// type
        assertEquals((byte) 3, b[11]); // length
        assertEquals((byte) 98, b[12]);// "b"
        assertEquals((byte) 49, b[13]);// "1"
        assertEquals((byte) 54, b[14]);// "6"
        assertEquals((byte) 0, b[15]);// key
        assertEquals((byte) 17, b[16]);// key
        assertEquals((byte) 1, b[17]);// type
        assertEquals((byte) 3, b[18]);// length
        assertEquals((byte) 98, b[19]);// "b"
        assertEquals((byte) 49, b[20]);// "1"
        assertEquals((byte) 55, b[21]);// "7"
    }

    @Test
    void testMultiToBuffer() {
        UdpPrimitiveLogger2 l = new UdpPrimitiveLogger2((x) -> bb = x, x -> mb = x);
        StringLogger s = l.stringLogger("label");
        s.log("hello");
        IntLogger i = l.intLogger("foo");
        i.log(1);
        l.flush();
        assertEquals(24, bb.remaining());
        byte[] b = bb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]); // key MSB
        assertEquals((byte) 17, b[9]); // key LSB
        assertEquals((byte) 3, b[10]); // int type
        assertEquals((byte) 0, b[11]); // int value
        assertEquals((byte) 0, b[12]); // int value
        assertEquals((byte) 0, b[13]); // int value
        assertEquals((byte) 1, b[14]); // int value
        assertEquals((byte) 0, b[15]); // key high byte
        assertEquals((byte) 16, b[16]); // key low byte
        assertEquals((byte) 6, b[17]); // string type
        assertEquals((byte) 5, b[18]); // length
        assertEquals((byte) 104, b[19]); // "h"
        assertEquals((byte) 101, b[20]);// "e"
        assertEquals((byte) 108, b[21]);// "l"
        assertEquals((byte) 108, b[22]);// "l"
        assertEquals((byte) 111, b[23]);// "o"

        // this should fill the buffer with the label
        l.dumpLabels();
        assertEquals(24, mb.remaining());
        b = mb.array();
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // ... timestamp
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 16, b[9]); // key
        assertEquals((byte) 6, b[10]); // type
        assertEquals((byte) 5, b[11]);// length
        assertEquals((byte) 108, b[12]);// "l"
        assertEquals((byte) 97, b[13]);// "a"
        assertEquals((byte) 98, b[14]);// "b"
        assertEquals((byte) 101, b[15]);// "e"
        assertEquals((byte) 108, b[16]);// "l"
        assertEquals((byte) 0, b[17]); //
        assertEquals((byte) 17, b[18]); // key
        assertEquals((byte) 3, b[19]); // type
        assertEquals((byte) 3, b[20]); // length
        assertEquals((byte) 102, b[21]);// "f"
        assertEquals((byte) 111, b[22]);// "o"
        assertEquals((byte) 111, b[23]);// "o"
    }
}
