package org.team100.lib.logging.primitive;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.ByteBuffer;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.primitive.UdpPrimitiveProtocol.ProtocolException;

class UdpPrimitiveProtocol2Test {

    //////////////////////////////
    //
    // single-type tests, encoding and decoding
    //

    @Test
    void testKey() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeBoolean(bb, 16, true);
        assertEquals(4, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 1, b[4]); // type
        assertEquals((byte) 1, b[5]); // value
        assertEquals((byte) 0, b[6]); //

        bb.rewind();
        bb.position(2);
        int key = UdpPrimitiveProtocol.decodeKey(bb);
        assertEquals(4, bb.position());
        assertEquals(16, key);
    }

    @Test
    void testType() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeBoolean(bb, 16, true);
        assertEquals(4, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 1, b[4]); // type
        assertEquals((byte) 1, b[5]); // value
        assertEquals((byte) 0, b[6]); //

        bb.rewind();
        bb.position(4);
        UdpType type = UdpPrimitiveProtocol.decodeType(bb);
        assertEquals(5, bb.position());
        assertEquals(UdpType.BOOLEAN, type);
    }

    @Test
    void testBoolean() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeBoolean(bb, 16, true);
        assertEquals(4, len);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]);
        assertEquals((byte) 0, b[2]); // key high byte
        assertEquals((byte) 16, b[3]); // key low byte
        assertEquals((byte) 1, b[4]); // type
        assertEquals((byte) 1, b[5]); // value
        assertEquals((byte) 0, b[6]); //

        bb.rewind();
        bb.position(5);
        boolean v = UdpPrimitiveProtocol.decodeBoolean(bb);
        assertEquals(6, bb.position());
        assertTrue(v);
    }

    @Test
    void testDouble() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeDouble(bb, 16, 15);
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
        bb.position(5);
        double v = UdpPrimitiveProtocol.decodeDouble(bb);
        assertEquals(13, bb.position());
        assertEquals(15, v);
    }

    @Test
    void testDoubleBounds() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        assertThrows(UdpPrimitiveProtocol.ProtocolException.class,
                () -> UdpPrimitiveProtocol.decodeDouble(bb));
    }

    @Test
    void testInt() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeInt(bb, 16, 15);
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
        bb.position(5);
        int v = UdpPrimitiveProtocol.decodeInt(bb);
        assertEquals(9, bb.position());
        assertEquals(15, v);
    }

    @Test
    void testDoubleArray() throws ProtocolException {
        byte[] b = new byte[24];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeDoubleArray(bb, 16, new double[] { 1.0, 2.0 });
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
        bb.position(5);
        double[] v = UdpPrimitiveProtocol.decodeDoubleArray(bb);
        assertEquals(22, bb.position());
        assertArrayEquals(new double[] { 1.0, 2.0 }, v);
    }

    @Test
    void testLong() throws ProtocolException {
        byte[] b = new byte[16];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeLong(bb, 16, 15);
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
        bb.position(5);
        long v = UdpPrimitiveProtocol.decodeLong(bb);
        assertEquals(13, bb.position());
        assertEquals(15, v);

    }

    @Test
    void testString() throws ProtocolException {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        // encoder doesn't start at the beginning
        bb.position(2);
        int len = UdpPrimitiveProtocol.encodeString(bb, 16, "hello");
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
        bb.position(5);
        String v = UdpPrimitiveProtocol.decodeString(bb);
        assertEquals(11, bb.position());
        assertEquals("hello", v);
    }

    ////////////////////////////////////////////
    //
    // multi-type buffer

    @Test
    void testStateful() {
        UdpPrimitiveProtocol p = new UdpPrimitiveProtocol();
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
        // skip timestamp since it varies
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
        int len = UdpPrimitiveProtocol.encodeBoolean(bb, 16, true);
        assertEquals(0, len);
    }

    @Test
    void testIntOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol.encodeInt(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testLongOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol.encodeLong(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testDoubleOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol.encodeDouble(bb, 16, 2);
        assertEquals(0, len);
    }

    @Test
    void testDoubleArrayOverflow() {
        byte[] b = new byte[10];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol.encodeDoubleArray(bb, 16, new double[] { 1.0, 2.0 });
        assertEquals(0, len);
    }

    @Test
    void testStringOverflow() {
        byte[] b = new byte[12];
        ByteBuffer bb = ByteBuffer.wrap(b);
        bb.position(8);
        int len = UdpPrimitiveProtocol.encodeString(bb, 16, "hello");
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
        UdpPrimitiveProtocol.encodeBoolean(bb, 16, true);
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
    }

    @Test
    void testDanglingKeyDouble() {
        byte[] b = new byte[5];
        ByteBuffer bb = ByteBuffer.wrap(b);
        UdpPrimitiveProtocol.encodeDouble(bb, 16, 15);
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
        UdpPrimitiveProtocol.encodeInt(bb, 16, 1);
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
        UdpPrimitiveProtocol.encodeDoubleArray(bb, 16, new double[] { 1.0 });
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
        UdpPrimitiveProtocol.encodeLong(bb, 16, 1);
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
        UdpPrimitiveProtocol.encodeString(bb, 16, "hello");
        assertEquals((byte) 0, b[0]);
        assertEquals((byte) 0, b[1]); // << make sure the key is not here
        assertEquals((byte) 0, b[2]); //
        assertEquals((byte) 0, b[3]); //
        assertEquals((byte) 0, b[4]); //
    }
}
