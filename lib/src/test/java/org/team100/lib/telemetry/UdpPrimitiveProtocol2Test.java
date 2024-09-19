package org.team100.lib.telemetry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.junit.jupiter.api.Test;

class UdpPrimitiveProtocol2Test {

    int key;
    UdpType type;
    Boolean boolVal;
    Double doubleVal;
    Integer intVal;
    double[] doubleArrayVal;
    Long longVal;
    String stringVal;

    //////////////////////////////
    //
    // single-type tests, encoding and decoding
    //

    @Test
    void testKey() {
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

        bb.rewind();
        int offset = UdpPrimitiveProtocol2.decodeKey(bb, 2, k -> key = k);
        assertEquals(4, offset);
        assertEquals(16, key);
    }

    @Test
    void testType() {
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

        bb.rewind();
        int offset = UdpPrimitiveProtocol2.decodeType(bb, 4, t -> type = t);
        assertEquals(5, offset);
        assertEquals(UdpType.BOOLEAN, type);
    }

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

        bb.rewind();
        BiConsumer<Integer, Boolean> consumer = (k, v) -> {
            key = k;
            boolVal = v;
        };
        int offset = UdpPrimitiveProtocol2.decodeBoolean(bb, 2, consumer);
        assertEquals(-1, offset);
    }

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

        bb.rewind();
        int offset = UdpPrimitiveProtocol2.decodeDouble(bb, 5, v -> doubleVal = v);
        assertEquals(13, offset);
        assertEquals(15, doubleVal);

    }

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

        bb.rewind();
        BiConsumer<Integer, Integer> consumer = (k, v) -> {
            key = k;
            intVal = v;
        };
        int offset = UdpPrimitiveProtocol2.decodeInt(bb, 2, consumer);
        assertEquals(-1, offset);
    }

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

        bb.rewind();
        BiConsumer<Integer, double[]> consumer = (k, v) -> {
            key = k;
            doubleArrayVal = v;
        };
        int offset = UdpPrimitiveProtocol2.decodeDoubleArray(bb, 2, consumer);
        assertEquals(-1, offset);
    }

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

        bb.rewind();
        BiConsumer<Integer, Long> consumer = (k, v) -> {
            key = k;
            longVal = v;
        };
        int offset = UdpPrimitiveProtocol2.decodeLong(bb, 2, consumer);
        assertEquals(-1, offset);

    }

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

        bb.rewind();
        BiConsumer<Integer, String> consumer = (k, v) -> {
            key = k;
            stringVal = v;
        };
        int offset = UdpPrimitiveProtocol2.decodeString(bb, 2, consumer);
        assertEquals(-1, offset);
    }

    ////////////////////////////////////////////
    //
    // multi-type buffer

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

    ////////////////////////////////////////////
    //
    // Overflow tests

    @Test
    void testBooleanOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeBoolean(bb, 16, true);
        assertEquals(0, len);
    }

    @Test
    void testIntOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeInt(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testLongOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeLong(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testDoubleOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeDouble(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testDoubleArrayOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeDoubleArray(bb, 16, new double[] { 1.0, 2.0 });
        assertEquals(0, len);
    }

    @Test
    void testStringOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol2.encodeString(bb, 16, "hello");
        assertEquals(0, len);
    }

    ///////////////////////////////////////
    //
    // Malformed packets
    //
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
}
