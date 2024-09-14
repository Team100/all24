package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;

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

    ////////////// KEY

    @Test
    void testKey() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeKey(bb, 16);
        assertEquals(2, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // high byte
        assertEquals((byte) 16, b[3]); // low byte
        assertEquals((byte) 0, b[4]);
    }

    @Test
    void testKeyMax() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);
        int len = UdpPrimitiveProtocol2.encodeKey(bb, 65535);
        assertEquals(2, len);
        assertEquals((byte) 0xFF, b[0]); // high byte
        assertEquals((byte) 0xFF, b[1]); // low byte
        assertEquals((byte) 0, b[2]);
    }

    @Test
    void testKeyBad() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);
        assertThrows(IllegalArgumentException.class, //
                () -> UdpPrimitiveProtocol2.encodeKey(bb, 1000000));
    }

    @Test
    void testKeyBufferFull() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);
        // position at the last byte
        bb.position(11);
        int len = UdpPrimitiveProtocol2.encodeKey(bb, 65535);
        // return zero means we didn't write anything, buffer is full.
        assertEquals(0, len);
        // really didn't write anything
        assertEquals((byte) 0, b[11]);
    }

    ///////////////////// STRING

    @Test
    void testString() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals(8, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 5, b[4]); // length
        assertEquals((byte) 104, b[5]); // h
        assertEquals((byte) 101, b[6]); // e
        assertEquals((byte) 108, b[7]); // l
        assertEquals((byte) 108, b[8]); // l
        assertEquals((byte) 111, b[9]); // o
        assertEquals((byte) 0, b[10]);
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
        assertEquals(10, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 0, b[4]); // value MSB
        assertEquals((byte) 0, b[5]); //
        assertEquals((byte) 0, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 15, b[11]); // value LSB
        assertEquals((byte) 0, b[12]);
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
        assertEquals(6, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 0, b[4]); // value MSB
        assertEquals((byte) 0, b[5]); //
        assertEquals((byte) 0, b[6]); //
        assertEquals((byte) 15, b[7]); // value LSB
        assertEquals((byte) 0, b[8]);
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
        assertEquals(10, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 64, b[4]); // value MSB
        assertEquals((byte) 46, b[5]); //
        assertEquals((byte) 0, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 0, b[11]); // value LSB
        assertEquals((byte) 0, b[12]);
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
        assertEquals(3, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 1, b[4]); // value
        assertEquals((byte) 0, b[5]); //
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
        assertEquals(19, len); // key (2) length (1) data (2*8)
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 2, b[4]); // length
        assertEquals((byte) 63, b[5]); // val MSB
        assertEquals((byte) -16, b[6]); //
        assertEquals((byte) 0, b[7]); //
        assertEquals((byte) 0, b[8]); //
        assertEquals((byte) 0, b[9]); //
        assertEquals((byte) 0, b[10]); //
        assertEquals((byte) 0, b[11]); //
        assertEquals((byte) 0, b[12]); // val LSB
        assertEquals((byte) 64, b[13]); // val MSB
        assertEquals((byte) 0, b[14]); //
        assertEquals((byte) 0, b[15]); //
        assertEquals((byte) 0, b[16]); //
        assertEquals((byte) 0, b[17]); //
        assertEquals((byte) 0, b[18]); //
        assertEquals((byte) 0, b[19]); //
        assertEquals((byte) 0, b[20]); // val LSB
        assertEquals((byte) 0, b[21]); //
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

    //////////////////// LABEL MAP

    @Test
    void testLabelMap() {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(2);
        List<String> labels = List.of("one", "two", "three", "four");
        int n = UdpPrimitiveProtocol2.encodeLabels(bb, 0, labels);
        assertEquals(2, n); // both labels are consumed
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // offset high byte
        assertEquals((byte) 0, b[3]); // offset low byte
        assertEquals((byte) 2, b[4]); // number of labels
        assertEquals((byte) 3, b[5]); // length
        assertEquals((byte) 111, b[6]); // o
        assertEquals((byte) 110, b[7]); // n
        assertEquals((byte) 101, b[8]); // e
        assertEquals((byte) 3, b[9]); // length 
        assertEquals((byte) 116, b[10]); // t
        assertEquals((byte) 119, b[11]); // w
        assertEquals((byte) 111, b[12]); // o
        assertEquals((byte) 0, b[13]); // 
    }

    @Test
    void testOffsetLabelMap() {
        byte[] b = new byte[24];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(2);
        List<String> labels = List.of("one", "two", "three", "four");
        int n = UdpPrimitiveProtocol2.encodeLabels(bb, 2, labels);
        assertEquals(2, n); // both labels are consumed
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // offset high byte
        assertEquals((byte) 2, b[3]); // offset low byte
        assertEquals((byte) 2, b[4]); // number of labels
        assertEquals((byte) 5, b[5]); // length
        assertEquals((byte) 116, b[6]); // t
        assertEquals((byte) 104, b[7]); // h
        assertEquals((byte) 114, b[8]); // r
        assertEquals((byte) 101, b[9]); // e
        assertEquals((byte) 101, b[10]); // e
        assertEquals((byte) 4, b[11]); // length
        assertEquals((byte) 102, b[12]); // f
        assertEquals((byte) 111, b[13]); // o
        assertEquals((byte) 117, b[14]); // u
        assertEquals((byte) 114, b[15]); // r
        assertEquals((byte) 0, b[16]); //
    }

    @Test
    void testFragmentedLabelMap() {
        byte[] b = new byte[11];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(2);
        List<String> labels = List.of("one", "two", "three", "four");
        int n = UdpPrimitiveProtocol2.encodeLabels(bb, 2, labels);
        assertEquals(1, n); // just one label consumed
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // offset high byte
        assertEquals((byte) 2, b[3]); // offset low byte
        assertEquals((byte) 1, b[4]); // number of labels
        assertEquals((byte) 5, b[5]); // length
        assertEquals((byte) 116, b[6]); // t
        assertEquals((byte) 104, b[7]); // h
        assertEquals((byte) 114, b[8]); // r
        assertEquals((byte) 101, b[9]); // e
        assertEquals((byte) 101, b[10]); // e
    }

    @Test
    void testLabelMapOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        List<String> labels = List.of("one", "two", "three", "four");
        int len = UdpPrimitiveProtocol2.encodeLabels(bb, 2, labels);
        assertEquals(0, len);
    }

    /////////////////// TYPE

    @Test
    void testType() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol2.encodeType(bb, UdpPrimitiveProtocol2.Type.INT);
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
        int len = UdpPrimitiveProtocol2.encodeType(bb, UdpPrimitiveProtocol2.Type.INT);
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
        assertEquals(10, p.buffer().position());
        assertTrue(p.putDouble(17, 1.0));
        // 2 for type, 2 for key, 8 for double, 12 bytes + 10 = position 22
        assertEquals(22, p.buffer().position());
        ByteBuffer bb = p.buffer();
        byte[] b = bb.array();
        assertEquals(508, b.length);
        assertEquals((byte) 0, b[0]); // string type
        assertEquals((byte) 6, b[1]); // string type
        assertEquals((byte) 0, b[2]); // key MSB
        assertEquals((byte) 16, b[3]); // key LSB
        assertEquals((byte) 5, b[4]); // value length
        assertEquals((byte) 104, b[5]); // "h"
        assertEquals((byte) 101, b[6]);// "e"
        assertEquals((byte) 108, b[7]);// "l"
        assertEquals((byte) 108, b[8]);// "l"
        assertEquals((byte) 111, b[9]);// "o"
        assertEquals((byte) 0, b[10]); // double type
        assertEquals((byte) 2, b[11]); // double type
        assertEquals((byte) 0, b[12]); // key MSB
        assertEquals((byte) 17, b[13]); // key LSB
        assertEquals((byte) 63, b[14]);
        assertEquals((byte) -16, b[15]);
        assertEquals((byte) 0, b[16]);
        assertEquals((byte) 0, b[17]);
        assertEquals((byte) 0, b[18]);
        assertEquals((byte) 0, b[19]);
        assertEquals((byte) 0, b[20]);
        assertEquals((byte) 0, b[21]);
    }

}
