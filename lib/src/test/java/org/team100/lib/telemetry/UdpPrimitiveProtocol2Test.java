package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.Arrays;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Timer;

class UdpPrimitiveProtocol2Test {
    @Test
    void testBuffer() {
        // are byte arrays initialized to zero?
        byte[] b = new byte[4];
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
    }

    ///////////////////// STRING

    @Test
    void testString() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals(9, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 6, b[4]); // type
        assertEquals((byte) 5, b[5]); // length
        assertEquals((byte) 104, b[6]); // h
        assertEquals((byte) 101, b[7]); // e
        assertEquals((byte) 108, b[8]); // l
        assertEquals((byte) 108, b[9]); // l
        assertEquals((byte) 111, b[10]); // o
        assertEquals((byte) 0, b[11]);
    }

    @Test
    void testStringOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals(0, len);
    }

    ////////////// LONG

    @Test
    void testLong() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeLong(bb, 16, 15);
        assertEquals(11, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 5, b[4]); // type
        assertEquals((byte) 0, b[5]); // value MSB
        assertEquals((byte) 0, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 0, b[11]); //
        assertEquals((byte) 15, b[12]); // value LSB
        assertEquals((byte) 0, b[13]);
    }

    @Test
    void testLongOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeLong(bb, 16, 2);
        assertEquals(0, len);
    }

    //////////// INT

    @Test
    void testInt() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeInt(bb, 16, 15);
        assertEquals(7, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 3, b[4]); // type
        assertEquals((byte) 0, b[5]); // value MSB
        assertEquals((byte) 0, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 15, b[8]); // value LSB
        assertEquals((byte) 0, b[9]);
    }

    @Test
    void testIntOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeInt(bb, 16, 2);
        assertEquals(0, len);
    }

    /////////////// DOUBLE

    @Test
    void testDouble() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeDouble(bb, 16, 15);
        assertEquals(11, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 2, b[4]); // type
        assertEquals((byte) 64, b[5]); // value MSB
        assertEquals((byte) 46, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 0, b[11]); //
        assertEquals((byte) 0, b[12]); // value LSB
        assertEquals((byte) 0, b[13]);
    }

    // dangling key tests
    // make sure we don't write a key with no value
    // a packet of doubles is 2 bytes for type, then 10 bytes per key+double
    // so the last one ends at 502 out of 508, leaving room for the key
    // but not the value.

    @Test
    void testDanglingKeyBoolean() {
        byte[] b = new byte[2]; // just room for the key
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeBoolean(bb, 16, true);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
    }

    @Test
    void testDanglingKeyDouble() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeDouble(bb, 16, 15);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }

    @Test
    void testDanglingKeyInt() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeInt(bb, 16, 1);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }

    @Test
    void testDanglingKeyDoubleArray() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeDoubleArray(bb, 16, new double[] { 1.0 });
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }

    @Test
    void testDanglingKeyLong() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeLong(bb, 16, 1);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }

    @Test
    void testDanglingKeyString() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }

    @Test
    void testDoubleOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeDouble(bb, 16, 2);
        assertEquals(0, len);
    }

    ///////////////////

    @Test
    void testBoolean() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeBoolean(bb, 16, true);
        assertEquals(4, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 1, b[4]); // type
        assertEquals((byte) 1, b[5]); // value
        assertEquals((byte) 0, b[6]); //
    }

    @Test
    void testBooleanOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeBoolean(bb, 16, true);
        assertEquals(0, len);
    }

    /////////////////////

    @Test
    void testDoubleArray() {
        byte[] b = new byte[24];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeDoubleArray(bb, 16, new double[] { 1.0, 2.0 });
        assertEquals(20, len); // key (2) length (1) data (2*8)
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 4, b[4]); // type
        assertEquals((byte) 2, b[5]); // length
        assertEquals((byte) 63, b[6]); // val MSB
        assertEquals((byte) -16, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 0, b[11]); //
        assertEquals((byte) 0, b[12]); //
        assertEquals((byte) 0, b[13]); // val LSB
        assertEquals((byte) 64, b[14]); // val MSB
        assertEquals((byte) 0, b[15]); //
        assertEquals((byte) 0, b[16]); //
        assertEquals((byte) 0, b[17]); //
        assertEquals((byte) 0, b[18]); //
        assertEquals((byte) 0, b[19]); //
        assertEquals((byte) 0, b[20]); //
        assertEquals((byte) 0, b[21]); // val LSB
        assertEquals((byte) 0, b[22]); //
        assertEquals((byte) 0, b[23]); //
    }

    @Test
    void testDoubleArrayOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeDoubleArray(bb, 16, new double[] { 1.0, 2.0 });
        assertEquals(0, len);
    }

    /////////////////// TYPE

    @Test
    void testType() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeType(bb, UdpType.INT);
        assertEquals(2, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 3, b[3]);
    }

    @Test
    void testTypeOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(10); // off the end
        int len = UdpPrimitiveProtocol2.encodeType(bb, UdpType.INT);
        assertEquals(0, len);
    }

    @Test
    void testBufferPerformance() {
        final int N = 10000000;
        final int S = 500;
        {
            // 130 ms (!)
            double t0 = Timer.getFPGATimestamp();
            byte[] b = new byte[S];
            for (int i = 0; i < N; ++i) {
                Arrays.fill(b, (byte) 0);
                b[0] = (byte) 1;
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("array fill duration (ms) %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("array fill per op (ns)   %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        // the methods below are all about the same -- it gets a little faster by the
        // third case, but that's not a real effect
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                byte[] b = new byte[S];
                b[0] = (byte) 1;
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new array duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new array per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                byte[] b = new byte[S];
                ByteBuffer bb = ByteBuffer.wrap(b);
                bb.put((byte) 1);
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("buf wrap duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("buf wrap per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 15 ms
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                ByteBuffer b = ByteBuffer.allocate(S);
                b.put((byte) 1);
            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new buf duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new buf per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
        {
            // 7 seconds (!!)
            // allocateDirect is *very* slow, 700 times slower.
            // use this only for singleton buffers.
            double t0 = Timer.getFPGATimestamp();
            for (int i = 0; i < N; ++i) {
                ByteBuffer b = ByteBuffer.allocateDirect(S);
                b.put((byte) 1);

            }
            double t1 = Timer.getFPGATimestamp();
            System.out.printf("new buf duration (ms)  %5.1f\n", 1e3 * (t1 - t0));
            System.out.printf("new buf per op (ns)    %5.1f\n", 1e9 * (t1 - t0) / N);

        }
    }

    @Test
    void testStateful() {
        UdpPrimitiveProtocol2 p = new UdpPrimitiveProtocol2();
        assertTrue(p.putString(16, "hello"));
        // 2 for type, 2 for key, 1 for length, string length 5 = 10 bytes
        assertEquals(17, p.buffer().position());
        assertTrue(p.putDouble(17, 1.0));
        // 2 for type, 2 for key, 8 for double, 12 bytes + 10 = position 22
        assertEquals(28, p.buffer().position());
        ByteBuffer bb = p.buffer();
        byte[] b = new byte[28];
        bb.rewind();
        bb.get(b);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]);
        assertEquals((byte) 0, b[3]);
        assertEquals((byte) 0, b[4]);
        assertEquals((byte) 0, b[5]);
        assertEquals((byte) 0, b[6]);
        assertEquals((byte) 0, b[7]); // timestamp
        assertEquals((byte) 0, b[8]); // key MSB
        assertEquals((byte) 16, b[9]); // key LSB
        assertEquals((byte) 6, b[10]); // string type
        assertEquals((byte) 5, b[11]); // value length
        assertEquals((byte) 104, b[12]); // "h"
        assertEquals((byte) 101, b[13]);// "e"
        assertEquals((byte) 108, b[14]);// "l"
        assertEquals((byte) 108, b[15]);// "l"
        assertEquals((byte) 111, b[16]);// "o"
        assertEquals((byte) 0, b[17]); // key MSB
        assertEquals((byte) 17, b[18]); // key LSB
        assertEquals((byte) 2, b[19]); // double type
        assertEquals((byte) 63, b[20]);
        assertEquals((byte) -16, b[21]);
        assertEquals((byte) 0, b[22]);
        assertEquals((byte) 0, b[23]);
        assertEquals((byte) 0, b[24]);
        assertEquals((byte) 0, b[25]);
        assertEquals((byte) 0, b[26]);
        assertEquals((byte) 0, b[27]);
    }

    /**
     * Encoding itself is quite fast, 4 ns per key, about 1 ns
     * per int on my machine, which is about as fast as it can go, I think.
     */
    @Test
    void testEncodingPerformance() throws Exception {
        final int ITERATIONS = 200000000;
        final int BUFFER_SIZE = 10000000;

        // huge buffer for this test; we're testing the encoding.
        UdpPrimitiveProtocol2 p = new UdpPrimitiveProtocol2(BUFFER_SIZE);

        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putBoolean(17, true))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("boolean duration sec %.3f\n", t2 - t1);
            System.out.printf("boolean duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }

        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDouble(17, 1.0))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("double duration sec %.3f\n", t2 - t1);
            System.out.printf("double duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putInt(17, 1))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("int duration sec %.3f\n", t2 - t1);
            System.out.printf("int duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDoubleArray(17, new double[] { 1.0 }))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("double[] duration sec %.3f\n", t2 - t1);
            System.out.printf("double[] duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putLong(17, 1))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("long duration sec %.3f\n", t2 - t1);
            System.out.printf("long duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
        {
            p.clear();
            double t1 = Timer.getFPGATimestamp();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putString(17, "asdf"))
                    p.clear();
            }
            double t2 = Timer.getFPGATimestamp();
            System.out.printf("string duration sec %.3f\n", t2 - t1);
            System.out.printf("string duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
        }
    }

    @Test
    void testDoubleEncodingPerformance() {
        // this goes at 1.3 ns/row, with or without offset
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        double t1 = Timer.getFPGATimestamp();
        for (int i = 0; i < ITERATIONS; ++i) {
            bb.clear();
            for (int j = 0; j < N; ++j) {
                bb.putDouble(j, j * 8);
            }
        }
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }

    @Test
    void testDoubleDecodingPerformance() {
        // 1 ns/row, including the addition
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        for (int i = 0; i < N; ++i) {
            bb.putDouble(i);
        }
        double t1 = Timer.getFPGATimestamp();
        double total = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            bb.rewind();
            for (int j = 0; j < N; ++j) {
                // some real work to prevent optimizing it all away
                total += bb.getDouble();
            }
        }
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("total %f\n", total);
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }

    @Test
    void testDoubleBulkDecodingPerformance() {
        // 1 ns/row, including the addition
        // bulk copying to a new double[] is >3ns, bad.
        final int ITERATIONS = 10000000;
        final int N = 180;
        ByteBuffer bb = ByteBuffer.allocateDirect(N * 8);
        for (int i = 0; i < N; ++i) {
            bb.putDouble(i);
        }
        bb.rewind();
        DoubleBuffer db = bb.asDoubleBuffer();
        double t1 = Timer.getFPGATimestamp();
        double total = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            db.rewind();
            for (int j = 0; j < N; ++j) {
                // some real work to prevent optimizing it all away
                total += db.get();
            }
        }
        System.out.printf("total %f\n", total);
        double t2 = Timer.getFPGATimestamp();
        System.out.printf("string duration sec %.3f\n", t2 - t1);
        System.out.printf("string duration per row ns %.3f\n", 1e9 * (t2 - t1) / (ITERATIONS * N));
    }

}
